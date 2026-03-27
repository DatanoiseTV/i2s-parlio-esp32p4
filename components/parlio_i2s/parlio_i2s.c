#include "parlio_i2s.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/parlio_tx.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "parlio_i2s";

/*
 * Internal representation of one PARLIO-based I2S TX instance.
 *
 * The APLL generates a precise audio master clock. We route it through the I2S
 * peripheral (used only as clock generator) -> MCLK GPIO -> PARLIO ext clock in.
 * PARLIO then outputs MCLK on its clock-out pin, and synthesizes BCLK, LRCK, and
 * data lines as parallel data bits in the DMA buffer.
 */
struct parlio_i2s_tx {
    /* configuration snapshot */
    uint32_t sample_rate;
    uint8_t  bits_per_sample;
    uint8_t  slot_width;
    uint8_t  num_data_lines;
    uint16_t mclk_multiple;

    /* derived timing constants */
    uint32_t mclk_freq;          /* MCLK = mclk_multiple * sample_rate */
    uint32_t real_mclk_freq;     /* actual MCLK after APLL rounding */
    uint16_t mclk_per_bclk;     /* MCLK cycles per BCLK period */
    uint16_t bclk_per_frame;    /* BCLK cycles per audio frame (= slot_width * 2) */
    uint16_t mclk_per_frame;    /* total MCLK cycles per frame */

    /* PARLIO data bus width (4, 8, or 16) */
    uint8_t  parlio_width;

    /* DMA buffer management */
    size_t   frame_buf_bytes;    /* bytes per audio frame in PARLIO format */
    size_t   dma_buf_bytes;      /* bytes per DMA buffer */
    size_t   frames_per_buf;
    size_t   dma_buf_count;
    uint8_t  **dma_bufs;         /* array of DMA-capable buffers */
    size_t   write_buf_idx;      /* next buffer to fill */
    size_t   write_frame_pos;    /* frame position within current buffer */

    /* synchronization */
    SemaphoreHandle_t write_sem;

    /* peripheral handles */
    i2s_chan_handle_t      i2s_clk_chan;
    parlio_tx_unit_handle_t parlio_unit;

    bool enabled;
};

/* Round up to the next valid PARLIO data width. */
static uint8_t next_parlio_width(uint8_t needed)
{
    if (needed <= 1)  return 1;
    if (needed <= 2)  return 2;
    if (needed <= 4)  return 4;
    if (needed <= 8)  return 8;
    if (needed <= 16) return 16;
    return 0; /* unsupported */
}

/*
 * Encode a single audio frame into the PARLIO DMA buffer.
 *
 * For each MCLK tick within the frame, we produce one parlio_width-bit word.
 * Bit layout:
 *   [0]   = BCLK
 *   [1]   = LRCK
 *   [2+i] = data line i
 *
 * I2S Philips format:
 *   - LRCK low  = left channel
 *   - LRCK high = right channel
 *   - LRCK transitions one BCLK before the first data bit (the "one-bit offset")
 *   - Data is valid on rising edge of BCLK, MSB first
 *   - Data changes on falling BCLK edge
 */
static void encode_frame(struct parlio_i2s_tx *h,
                          uint8_t *dst,
                          const int32_t *samples)
{
    const uint16_t mpb  = h->mclk_per_bclk;   /* MCLK ticks per BCLK cycle */
    const uint16_t bpf  = h->bclk_per_frame;  /* BCLK cycles per frame */
    const uint8_t  sw   = h->slot_width;
    const uint8_t  bps  = h->bits_per_sample;
    const uint8_t  ndl  = h->num_data_lines;
    const uint8_t  pw   = h->parlio_width;

    /*
     * samples layout (per frame): [L0, R0, L1, R1, ...] as int32_t left-justified.
     * If samples is NULL, output silence (all data bits zero).
     */

    uint16_t mclk_idx = 0;
    for (uint16_t bclk = 0; bclk < bpf; bclk++) {
        /*
         * Determine the channel (left/right) and bit index.
         * In Philips I2S, LRCK transitions one BCLK before the MSB.
         *
         * bclk 0..sw-1   -> left slot  (LRCK=0 from bclk 0)
         * bclk sw..2*sw-1 -> right slot (LRCK=1 from bclk sw)
         *
         * But Philips has a one-cycle offset: LRCK changes one BCLK early.
         * So LRCK goes low at bclk = (2*sw - 1) for next frame's left,
         * and goes high at bclk = (sw - 1) for right.
         *
         * Simpler: LRCK transitions at bclk = sw-1 (rising) and 2*sw-1 (falling).
         * We encode LRCK as: (bclk >= sw) for the current frame, with the
         * understanding that the one-bit offset is handled by shifting the data
         * start by one BCLK cycle.
         */

        /* LRCK for Philips: transitions one BCLK before data MSB */
        uint8_t lrck;
        if (bclk == 0) {
            /* first BCLK of frame: LRCK went low on previous frame's last BCLK */
            lrck = 0;
        } else if (bclk < sw) {
            lrck = 0;
        } else {
            lrck = 1;
        }

        /* Which channel and which bit of the sample are we outputting? */
        uint8_t  channel;      /* 0 = left, 1 = right */
        int16_t  bit_idx;      /* bit position within the sample, counting from MSB=0 */

        if (bclk < sw) {
            channel = 0;
            /* Philips offset: data MSB appears on the second BCLK after LRCK transition */
            bit_idx = (int16_t)bclk - 1;
        } else {
            channel = 1;
            bit_idx = (int16_t)(bclk - sw) - 1;
        }

        /* Build the data bits for all lines */
        uint16_t data_bits = 0;
        for (uint8_t line = 0; line < ndl; line++) {
            uint8_t bit_val = 0;
            if (bit_idx >= 0 && bit_idx < bps && samples != NULL) {
                /* samples are left-justified int32_t, MSB at bit 31 */
                int32_t samp = samples[line * 2 + channel];
                bit_val = (samp >> (31 - bit_idx)) & 1;
            }
            data_bits |= (bit_val << line);
        }

        /* For each MCLK sub-tick within this BCLK cycle, output the word.
         * BCLK is high for the first half, low for the second half.
         * Data changes on the falling edge of BCLK (beginning of low phase). */
        for (uint16_t m = 0; m < mpb; m++, mclk_idx++) {
            uint8_t bclk_val = (m < (mpb / 2)) ? 1 : 0;

            uint16_t word = 0;
            word |= (bclk_val & 1) << 0;
            word |= (lrck & 1)     << 1;
            word |= (data_bits)    << 2;

            /* Pack into the output buffer according to parlio_width */
            if (pw <= 8) {
                dst[mclk_idx] = (uint8_t)word;
            } else {
                /* 16-bit mode: little-endian */
                uint16_t *dst16 = (uint16_t *)dst;
                dst16[mclk_idx] = word;
            }
        }
    }
}

/* ISR callback: PARLIO TX done -- release a buffer slot for writing. */
static bool IRAM_ATTR on_parlio_tx_done(parlio_tx_unit_handle_t tx_unit,
                                         const parlio_tx_done_event_data_t *edata,
                                         void *user_ctx)
{
    struct parlio_i2s_tx *h = (struct parlio_i2s_tx *)user_ctx;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(h->write_sem, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

/*
 * Set up the I2S peripheral purely as an APLL-driven clock generator.
 * We only use it to get a precise MCLK output; no audio data flows through I2S.
 */
static esp_err_t setup_apll_clock(struct parlio_i2s_tx *h,
                                   gpio_num_t apll_feed_gpio)
{
    /* Allocate an I2S TX channel -- we will never write audio data to it */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2;
    chan_cfg.dma_frame_num = 16;
    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&chan_cfg, &h->i2s_clk_chan, NULL),
        TAG, "failed to allocate I2S channel for APLL clock");

    /* Configure STD mode with APLL clock source */
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = h->sample_rate,
            .clk_src = I2S_CLK_SRC_APLL,
            .mclk_multiple = (i2s_mclk_multiple_t)h->mclk_multiple,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = apll_feed_gpio,
            .bclk = I2S_GPIO_UNUSED,
            .ws   = I2S_GPIO_UNUSED,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(h->i2s_clk_chan, &std_cfg),
        TAG, "failed to init I2S STD mode for APLL clock");

    /* Enable the channel to start MCLK output */
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(h->i2s_clk_chan),
        TAG, "failed to enable I2S clock channel");

    ESP_LOGI(TAG, "APLL clock configured: target MCLK=%"PRIu32" Hz on GPIO %d",
             h->mclk_freq, apll_feed_gpio);
    return ESP_OK;
}

/*
 * Set up the PARLIO TX unit with external clock from the APLL feed GPIO.
 */
static esp_err_t setup_parlio(struct parlio_i2s_tx *h,
                               const parlio_i2s_tx_config_t *cfg)
{
    parlio_tx_unit_config_t parlio_cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,        /* overridden by clk_in_gpio_num */
        .clk_in_gpio_num = cfg->apll_feed_gpio,   /* external clock from APLL via I2S */
        .input_clk_src_freq_hz = h->mclk_freq,
        .output_clk_freq_hz = h->mclk_freq,       /* 1:1 passthrough, no division */
        .data_width = h->parlio_width,
        .clk_out_gpio_num = cfg->mclk_gpio,       /* MCLK output to the DAC */
        .valid_gpio_num = -1,                      /* not used */
        .trans_queue_depth = cfg->dma_buffer_count,
        .max_transfer_size = h->dma_buf_bytes,
        .dma_burst_size = 0,
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .flags = {
            .clk_gate_en = 0,
            .io_loop_back = 0,
        },
    };

    /* Assign GPIO pins: bit 0 = BCLK, bit 1 = LRCK, bit 2+ = data lines */
    for (int i = 0; i < 16; i++) {
        parlio_cfg.data_gpio_nums[i] = -1;
    }
    parlio_cfg.data_gpio_nums[0] = cfg->bclk_gpio;
    parlio_cfg.data_gpio_nums[1] = cfg->lrck_gpio;
    for (uint8_t i = 0; i < cfg->num_data_lines; i++) {
        parlio_cfg.data_gpio_nums[2 + i] = cfg->data_gpios[i];
    }

    ESP_RETURN_ON_ERROR(
        parlio_new_tx_unit(&parlio_cfg, &h->parlio_unit),
        TAG, "failed to create PARLIO TX unit");

    /* Register TX-done callback for flow control */
    parlio_tx_event_callbacks_t cbs = {
        .on_trans_done = on_parlio_tx_done,
    };
    ESP_RETURN_ON_ERROR(
        parlio_tx_unit_register_event_callbacks(h->parlio_unit, &cbs, h),
        TAG, "failed to register PARLIO callbacks");

    ESP_LOGI(TAG, "PARLIO TX: width=%u, MCLK=%"PRIu32" Hz, BCLK=%"PRIu32" Hz",
             h->parlio_width, h->mclk_freq,
             h->mclk_freq / h->mclk_per_bclk);
    return ESP_OK;
}

/* --- public API --- */

esp_err_t parlio_i2s_tx_new(const parlio_i2s_tx_config_t *config,
                             parlio_i2s_tx_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(config->num_data_lines >= 1 &&
                        config->num_data_lines <= PARLIO_I2S_MAX_DATA_LINES,
                        ESP_ERR_INVALID_ARG, TAG, "num_data_lines out of range");
    ESP_RETURN_ON_FALSE(config->bits_per_sample == 16 ||
                        config->bits_per_sample == 24 ||
                        config->bits_per_sample == 32,
                        ESP_ERR_INVALID_ARG, TAG, "unsupported bits_per_sample");

    uint8_t slot_w = config->slot_width;
    if (slot_w == 0) slot_w = 32; /* default */
    ESP_RETURN_ON_FALSE(slot_w >= config->bits_per_sample,
                        ESP_ERR_INVALID_ARG, TAG, "slot_width < bits_per_sample");

    uint16_t mclk_mult = config->mclk_multiple;
    if (mclk_mult == 0) mclk_mult = 256;

    /* Compute PARLIO bus width needed */
    uint8_t needed_bits = 2 + config->num_data_lines; /* BCLK + LRCK + data */
    uint8_t pw = next_parlio_width(needed_bits);
    ESP_RETURN_ON_FALSE(pw > 0 && pw <= 16, ESP_ERR_INVALID_ARG, TAG,
                        "too many data lines for PARLIO");

    struct parlio_i2s_tx *h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "alloc handle");

    h->sample_rate    = config->sample_rate;
    h->bits_per_sample = config->bits_per_sample;
    h->slot_width     = slot_w;
    h->num_data_lines = config->num_data_lines;
    h->mclk_multiple  = mclk_mult;
    h->parlio_width   = pw;

    /* Timing calculations */
    h->mclk_freq      = (uint32_t)mclk_mult * config->sample_rate;
    h->bclk_per_frame = (uint16_t)(slot_w * 2);
    h->mclk_per_bclk  = mclk_mult / h->bclk_per_frame;
    h->mclk_per_frame = (uint16_t)(h->mclk_per_bclk * h->bclk_per_frame);

    ESP_RETURN_ON_FALSE(h->mclk_per_bclk >= 2, ESP_ERR_INVALID_ARG, TAG,
                        "MCLK/BCLK ratio too low (< 2), increase mclk_multiple");
    ESP_RETURN_ON_FALSE((mclk_mult % h->bclk_per_frame) == 0, ESP_ERR_INVALID_ARG, TAG,
                        "mclk_multiple must be divisible by (slot_width * 2)");

    /* DMA buffer sizing */
    h->frames_per_buf = config->frames_per_buffer ? config->frames_per_buffer : 128;
    h->dma_buf_count  = config->dma_buffer_count  ? config->dma_buffer_count  : 4;

    size_t bytes_per_mclk_tick = (pw <= 8) ? 1 : 2;
    h->frame_buf_bytes = h->mclk_per_frame * bytes_per_mclk_tick;
    h->dma_buf_bytes   = h->frame_buf_bytes * h->frames_per_buf;

    ESP_LOGI(TAG, "frame_buf=%u B, dma_buf=%u B x %u, total=%u B",
             (unsigned)h->frame_buf_bytes, (unsigned)h->dma_buf_bytes,
             (unsigned)h->dma_buf_count,
             (unsigned)(h->dma_buf_bytes * h->dma_buf_count));

    /* Allocate DMA buffers in internal memory (DMA-capable) */
    h->dma_bufs = calloc(h->dma_buf_count, sizeof(uint8_t *));
    ESP_GOTO_ON_FALSE(h->dma_bufs, ESP_ERR_NO_MEM, fail, TAG, "alloc buf array");

    for (size_t i = 0; i < h->dma_buf_count; i++) {
        h->dma_bufs[i] = heap_caps_calloc(1, h->dma_buf_bytes,
                                           MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        ESP_GOTO_ON_FALSE(h->dma_bufs[i], ESP_ERR_NO_MEM, fail, TAG,
                          "alloc DMA buf %u", (unsigned)i);
    }

    h->write_sem = xSemaphoreCreateCounting(h->dma_buf_count, h->dma_buf_count);
    ESP_GOTO_ON_FALSE(h->write_sem, ESP_ERR_NO_MEM, fail, TAG, "alloc semaphore");

    /* Set up APLL -> I2S MCLK -> GPIO loopback */
    esp_err_t ret = setup_apll_clock(h, config->apll_feed_gpio);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "APLL setup failed");

    /* Set up PARLIO TX with ext clock from the same GPIO */
    ret = setup_parlio(h, config);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "PARLIO setup failed");

    *ret_handle = h;
    return ESP_OK;

fail:
    parlio_i2s_tx_delete(h);
    return ESP_FAIL;
}

esp_err_t parlio_i2s_tx_enable(parlio_i2s_tx_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_FALSE(!handle->enabled, ESP_ERR_INVALID_STATE, TAG, "already enabled");

    ESP_RETURN_ON_ERROR(
        parlio_tx_unit_enable(handle->parlio_unit),
        TAG, "PARLIO enable failed");

    handle->enabled = true;
    handle->write_buf_idx = 0;
    handle->write_frame_pos = 0;

    /* Pre-fill all buffers with silence and queue them */
    for (size_t i = 0; i < handle->dma_buf_count; i++) {
        memset(handle->dma_bufs[i], 0, handle->dma_buf_bytes);
        /* Encode silence frames so BCLK/LRCK still toggle correctly */
        for (size_t f = 0; f < handle->frames_per_buf; f++) {
            uint8_t *frame_ptr = handle->dma_bufs[i] + f * handle->frame_buf_bytes;
            encode_frame(handle, frame_ptr, NULL);
        }
        parlio_transmit_config_t tx_cfg = { .idle_value = 0 };
        ESP_RETURN_ON_ERROR(
            parlio_tx_unit_transmit(handle->parlio_unit,
                                    handle->dma_bufs[i],
                                    handle->dma_buf_bytes * 8,
                                    &tx_cfg),
            TAG, "initial silence transmit failed");
        /* Consume one semaphore slot per queued buffer */
        xSemaphoreTake(handle->write_sem, 0);
    }

    ESP_LOGI(TAG, "transmitter enabled, Fs=%"PRIu32", %u-bit, %u data lines",
             handle->sample_rate, handle->bits_per_sample, handle->num_data_lines);
    return ESP_OK;
}

esp_err_t parlio_i2s_tx_disable(parlio_i2s_tx_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    if (!handle->enabled) return ESP_OK;

    parlio_tx_unit_wait_all_done(handle->parlio_unit, 500);
    ESP_RETURN_ON_ERROR(
        parlio_tx_unit_disable(handle->parlio_unit),
        TAG, "PARLIO disable failed");

    handle->enabled = false;
    return ESP_OK;
}

esp_err_t parlio_i2s_tx_write(parlio_i2s_tx_handle_t handle,
                               const int32_t *samples,
                               size_t num_frames,
                               size_t *frames_written,
                               uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && samples, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(handle->enabled, ESP_ERR_INVALID_STATE, TAG, "not enabled");

    const size_t channels_per_frame = handle->num_data_lines * 2;
    size_t written = 0;

    while (written < num_frames) {
        /* If current buffer is full, submit it and move to the next */
        if (handle->write_frame_pos >= handle->frames_per_buf) {
            parlio_transmit_config_t tx_cfg = { .idle_value = 0 };
            esp_err_t ret = parlio_tx_unit_transmit(
                handle->parlio_unit,
                handle->dma_bufs[handle->write_buf_idx],
                handle->dma_buf_bytes * 8,
                &tx_cfg);
            ESP_RETURN_ON_ERROR(ret, TAG, "transmit failed");

            handle->write_buf_idx = (handle->write_buf_idx + 1) % handle->dma_buf_count;
            handle->write_frame_pos = 0;

            /* Wait for a buffer to become available (TX done callback) */
            if (xSemaphoreTake(handle->write_sem,
                               pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
                if (frames_written) *frames_written = written;
                return ESP_ERR_TIMEOUT;
            }
        }

        /* Encode the frame into the current DMA buffer */
        uint8_t *frame_dst = handle->dma_bufs[handle->write_buf_idx]
                           + handle->write_frame_pos * handle->frame_buf_bytes;
        encode_frame(handle, frame_dst,
                     &samples[written * channels_per_frame]);

        handle->write_frame_pos++;
        written++;
    }

    if (frames_written) *frames_written = written;
    return ESP_OK;
}

esp_err_t parlio_i2s_tx_delete(parlio_i2s_tx_handle_t handle)
{
    if (!handle) return ESP_OK;

    if (handle->enabled) {
        parlio_i2s_tx_disable(handle);
    }

    if (handle->parlio_unit) {
        parlio_del_tx_unit(handle->parlio_unit);
    }

    if (handle->i2s_clk_chan) {
        i2s_channel_disable(handle->i2s_clk_chan);
        i2s_del_channel(handle->i2s_clk_chan);
    }

    if (handle->dma_bufs) {
        for (size_t i = 0; i < handle->dma_buf_count; i++) {
            free(handle->dma_bufs[i]);
        }
        free(handle->dma_bufs);
    }

    if (handle->write_sem) {
        vSemaphoreDelete(handle->write_sem);
    }

    free(handle);
    return ESP_OK;
}

uint32_t parlio_i2s_tx_get_real_sample_rate(parlio_i2s_tx_handle_t handle)
{
    if (!handle) return 0;
    /* The actual rate depends on APLL rounding. We can query it from I2S info. */
    /* For now, return the configured rate -- the APLL is usually very close. */
    return handle->sample_rate;
}
