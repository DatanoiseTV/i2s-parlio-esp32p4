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
 * Architecture:
 *
 *   APLL -> I2S MCLK output (mclk_gpio) -> PARLIO ext clock input
 *   PARLIO divides MCLK to BCLK internally (integer divider)
 *   PARLIO clk_out pin = BCLK (dedicated, does not consume a data line)
 *   PARLIO TXD[0]      = LRCK / frame sync
 *   PARLIO TXD[1..N]   = audio data
 *
 * Because PARLIO runs at BCLK rate, the DMA buffer contains exactly one
 * parallel word per BCLK cycle. This is much more efficient than the MCLK-rate
 * approach which required mclk_per_bclk words per BCLK cycle.
 *
 * The mclk_gpio pin serves dual purpose: it outputs MCLK to the DAC and
 * simultaneously feeds PARLIO as external clock input via the GPIO matrix.
 */
struct parlio_i2s_tx {
    /* configuration snapshot */
    uint32_t sample_rate;
    uint8_t  bits_per_sample;
    uint8_t  slot_width;
    uint8_t  num_data_lines;
    uint8_t  num_slots;
    uint16_t mclk_multiple;
    parlio_i2s_mode_t mode;

    /* derived timing constants */
    uint32_t mclk_freq;          /* MCLK = mclk_multiple * sample_rate */
    uint32_t bclk_freq;          /* BCLK = slot_width * num_slots * sample_rate */
    uint16_t bclk_per_frame;    /* BCLK cycles per audio frame (= slot_width * num_slots) */

    /* PARLIO data bus width (1, 2, 4, 8, or 16) */
    uint8_t  parlio_width;

    /* DMA buffer management */
    size_t   frame_buf_bytes;    /* bytes per audio frame in PARLIO format */
    size_t   dma_buf_bytes;      /* bytes per DMA buffer */
    size_t   frames_per_buf;
    size_t   dma_buf_count;
    uint8_t  **dma_bufs;
    size_t   write_buf_idx;
    size_t   write_frame_pos;

    /* synchronization */
    SemaphoreHandle_t write_sem;

    /* peripheral handles */
    i2s_chan_handle_t      i2s_clk_chan;
    parlio_tx_unit_handle_t parlio_unit;

    bool enabled;
};

static uint8_t next_parlio_width(uint8_t needed)
{
    if (needed <= 1)  return 1;
    if (needed <= 2)  return 2;
    if (needed <= 4)  return 4;
    if (needed <= 8)  return 8;
    if (needed <= 16) return 16;
    return 0;
}

static inline void write_parlio_word(uint8_t *dst, uint16_t idx,
                                      uint16_t word, uint8_t pw)
{
    if (pw <= 8) {
        dst[idx] = (uint8_t)word;
    } else {
        ((uint16_t *)dst)[idx] = word;
    }
}

/*
 * Encode a single audio frame into the PARLIO DMA buffer.
 *
 * PARLIO runs at BCLK rate, so we produce exactly one word per BCLK cycle.
 * Bit layout of each word:
 *   [0]     = LRCK / frame sync
 *   [1+i]   = data line i  (i = 0..num_data_lines-1)
 *
 * BCLK itself is output on the dedicated clk_out pin -- not in the data bus.
 *
 * I2S Philips (STANDARD) format:
 *   - LRCK low = left channel (slot 0), LRCK high = right channel (slot 1)
 *   - LRCK transitions one BCLK before the first data bit
 *   - Data MSB first, sampled on rising BCLK edge
 *
 * TDM format:
 *   - Frame sync: 1 BCLK cycle high at the start of each frame
 *   - Slots packed sequentially: slot0, slot1, ..., slotN
 *   - Data MSB first with one BCLK offset after slot boundary
 */
static void encode_frame(struct parlio_i2s_tx *h,
                          uint8_t *dst,
                          const int32_t *samples)
{
    const uint16_t bpf  = h->bclk_per_frame;
    const uint8_t  sw   = h->slot_width;
    const uint8_t  bps  = h->bits_per_sample;
    const uint8_t  ndl  = h->num_data_lines;
    const uint8_t  ns   = h->num_slots;
    const uint8_t  pw   = h->parlio_width;
    const bool     is_std = (h->mode == PARLIO_I2S_MODE_STANDARD);

    for (uint16_t bclk = 0; bclk < bpf; bclk++) {
        uint8_t slot = bclk / sw;
        uint16_t pos_in_slot = bclk % sw;

        /* LRCK / frame sync on bit 0 */
        uint8_t lrck;
        if (is_std) {
            /* Philips: low for slot 0, high for slot 1 */
            lrck = (slot >= 1) ? 1 : 0;
        } else {
            /* TDM: frame sync pulse high for one BCLK at frame start */
            lrck = (bclk == 0) ? 1 : 0;
        }

        /* Data bit index: one-BCLK offset, MSB first */
        int16_t bit_idx = (int16_t)pos_in_slot - 1;

        /* Build data bits for all lines on bits [1..ndl] */
        uint16_t data_bits = 0;
        if (bit_idx >= 0 && bit_idx < bps && samples != NULL) {
            for (uint8_t line = 0; line < ndl; line++) {
                int32_t samp = samples[line * ns + slot];
                uint8_t bit_val = (samp >> (31 - bit_idx)) & 1;
                data_bits |= (bit_val << line);
            }
        }

        /* Pack: bit 0 = LRCK, bits 1+ = data */
        uint16_t word = (lrck & 1) | (data_bits << 1);
        write_parlio_word(dst, bclk, word, pw);
    }
}

/* ISR callback: PARLIO TX done -- release a buffer slot for writing. */
static bool IRAM_ATTR on_parlio_tx_done(parlio_tx_unit_handle_t tx_unit,
                                         const parlio_tx_done_event_data_t *edata,
                                         void *user_ctx)
{
    struct parlio_i2s_tx *h = (struct parlio_i2s_tx *)user_ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(h->write_sem, &woken);
    return woken == pdTRUE;
}

/*
 * Set up the I2S peripheral purely as an APLL-driven clock generator.
 * Only MCLK output is used; no audio data flows through I2S.
 *
 * The mclk_gpio simultaneously serves as:
 *   1. MCLK output to the DAC (directly from I2S/APLL)
 *   2. PARLIO external clock input (via GPIO matrix)
 */
static esp_err_t setup_apll_clock(struct parlio_i2s_tx *h, gpio_num_t mclk_gpio)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2;
    chan_cfg.dma_frame_num = 16;
    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&chan_cfg, &h->i2s_clk_chan, NULL),
        TAG, "failed to allocate I2S channel for APLL clock");

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = h->sample_rate,
            .clk_src = I2S_CLK_SRC_APLL,
            .mclk_multiple = (i2s_mclk_multiple_t)h->mclk_multiple,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = mclk_gpio,
            .bclk = I2S_GPIO_UNUSED,
            .ws   = I2S_GPIO_UNUSED,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(h->i2s_clk_chan, &std_cfg),
        TAG, "failed to init I2S STD mode for APLL clock");

    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(h->i2s_clk_chan),
        TAG, "failed to enable I2S clock channel");

    ESP_LOGI(TAG, "APLL: MCLK=%"PRIu32" Hz on GPIO %d", h->mclk_freq, mclk_gpio);
    return ESP_OK;
}

/*
 * Set up the PARLIO TX unit.
 *
 * External clock = MCLK (from the mclk_gpio via GPIO matrix).
 * PARLIO divides MCLK down to BCLK for the output clock and data shift rate.
 * clk_out pin = BCLK, TXD[0] = LRCK, TXD[1..N] = audio data.
 */
static esp_err_t setup_parlio(struct parlio_i2s_tx *h,
                               const parlio_i2s_tx_config_t *cfg)
{
    parlio_tx_unit_config_t parlio_cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = cfg->mclk_gpio,        /* MCLK input from I2S/APLL */
        .input_clk_src_freq_hz = h->mclk_freq,    /* source = MCLK */
        .output_clk_freq_hz = h->bclk_freq,       /* PARLIO divides to BCLK */
        .data_width = h->parlio_width,
        .clk_out_gpio_num = cfg->bclk_gpio,       /* BCLK on dedicated clock pin */
        .valid_gpio_num = -1,
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

    /* TXD[0] = LRCK, TXD[1..N] = audio data, rest unused */
    for (int i = 0; i < 16; i++) {
        parlio_cfg.data_gpio_nums[i] = -1;
    }
    parlio_cfg.data_gpio_nums[0] = cfg->lrck_gpio;
    for (uint8_t i = 0; i < cfg->num_data_lines; i++) {
        parlio_cfg.data_gpio_nums[1 + i] = cfg->data_gpios[i];
    }

    ESP_RETURN_ON_ERROR(
        parlio_new_tx_unit(&parlio_cfg, &h->parlio_unit),
        TAG, "failed to create PARLIO TX unit");

    parlio_tx_event_callbacks_t cbs = {
        .on_trans_done = on_parlio_tx_done,
    };
    ESP_RETURN_ON_ERROR(
        parlio_tx_unit_register_event_callbacks(h->parlio_unit, &cbs, h),
        TAG, "failed to register PARLIO callbacks");

    ESP_LOGI(TAG, "PARLIO TX: width=%u, BCLK=%"PRIu32" Hz (MCLK/%u), "
             "%u slots/line, %u data lines, %u total channels",
             h->parlio_width, h->bclk_freq,
             (unsigned)(h->mclk_freq / h->bclk_freq),
             h->num_slots, h->num_data_lines,
             h->num_slots * h->num_data_lines);
    return ESP_OK;
}

static uint8_t resolve_num_slots(parlio_i2s_mode_t mode)
{
    switch (mode) {
    case PARLIO_I2S_MODE_STANDARD: return 2;
    case PARLIO_I2S_MODE_TDM4:    return 4;
    case PARLIO_I2S_MODE_TDM8:    return 8;
    case PARLIO_I2S_MODE_TDM16:   return 16;
    default:                       return 0;
    }
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

    parlio_i2s_mode_t mode = config->mode;
    if (mode == 0) mode = PARLIO_I2S_MODE_STANDARD;
    uint8_t num_slots = resolve_num_slots(mode);
    ESP_RETURN_ON_FALSE(num_slots > 0, ESP_ERR_INVALID_ARG, TAG,
                        "invalid mode, use STANDARD/TDM4/TDM8/TDM16");

    uint8_t slot_w = config->slot_width;
    if (slot_w == 0) slot_w = 32;
    ESP_RETURN_ON_FALSE(slot_w >= config->bits_per_sample,
                        ESP_ERR_INVALID_ARG, TAG, "slot_width < bits_per_sample");

    uint16_t mclk_mult = config->mclk_multiple;
    if (mclk_mult == 0) mclk_mult = 256;

    /* PARLIO bus: TXD[0] = LRCK, TXD[1..N] = data */
    uint8_t needed_bits = 1 + config->num_data_lines;
    uint8_t pw = next_parlio_width(needed_bits);
    ESP_RETURN_ON_FALSE(pw > 0 && pw <= 16, ESP_ERR_INVALID_ARG, TAG,
                        "too many data lines for PARLIO");

    struct parlio_i2s_tx *h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "alloc handle");
    esp_err_t ret = ESP_OK;

    h->sample_rate    = config->sample_rate;
    h->bits_per_sample = config->bits_per_sample;
    h->slot_width     = slot_w;
    h->num_data_lines = config->num_data_lines;
    h->num_slots      = num_slots;
    h->mclk_multiple  = mclk_mult;
    h->mode           = mode;
    h->parlio_width   = pw;

    /* Timing:
     * MCLK = mclk_multiple * sample_rate
     * BCLK = slot_width * num_slots * sample_rate
     * MCLK/BCLK ratio must be integer >= 1 */
    h->bclk_per_frame = (uint16_t)(slot_w * num_slots);
    h->mclk_freq      = (uint32_t)mclk_mult * config->sample_rate;
    h->bclk_freq      = (uint32_t)h->bclk_per_frame * config->sample_rate;

    uint32_t mclk_bclk_ratio = mclk_mult / h->bclk_per_frame;
    ESP_RETURN_ON_FALSE(mclk_bclk_ratio >= 1, ESP_ERR_INVALID_ARG, TAG,
                        "MCLK/BCLK ratio < 1, increase mclk_multiple "
                        "(need >= %u for %u slots x %u-bit)",
                        (unsigned)h->bclk_per_frame, num_slots, slot_w);
    ESP_RETURN_ON_FALSE((mclk_mult % h->bclk_per_frame) == 0, ESP_ERR_INVALID_ARG, TAG,
                        "mclk_multiple (%u) must be divisible by slot_width*num_slots (%u)",
                        mclk_mult, h->bclk_per_frame);

    /* DMA: one word per BCLK tick (not per MCLK tick anymore) */
    h->frames_per_buf = config->frames_per_buffer ? config->frames_per_buffer : 128;
    h->dma_buf_count  = config->dma_buffer_count  ? config->dma_buffer_count  : 4;

    size_t bytes_per_bclk_tick = (pw <= 8) ? 1 : 2;
    h->frame_buf_bytes = h->bclk_per_frame * bytes_per_bclk_tick;
    h->dma_buf_bytes   = h->frame_buf_bytes * h->frames_per_buf;

    ESP_LOGI(TAG, "mode=%s, %u slots/line, %u lines, %u ch, "
             "MCLK=%"PRIu32", BCLK=%"PRIu32" (ratio %u), "
             "frame=%u B, dma=%u B x %u",
             (mode == PARLIO_I2S_MODE_STANDARD) ? "I2S" :
             (mode == PARLIO_I2S_MODE_TDM4) ? "TDM4" :
             (mode == PARLIO_I2S_MODE_TDM8) ? "TDM8" : "TDM16",
             num_slots, config->num_data_lines,
             (unsigned)(num_slots * config->num_data_lines),
             h->mclk_freq, h->bclk_freq, (unsigned)mclk_bclk_ratio,
             (unsigned)h->frame_buf_bytes, (unsigned)h->dma_buf_bytes,
             (unsigned)h->dma_buf_count);

    /* Allocate DMA buffers */
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

    /* APLL -> I2S MCLK -> GPIO (output to DAC + input to PARLIO) */
    ret = setup_apll_clock(h, config->mclk_gpio);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "APLL setup failed");

    /* PARLIO: ext clk = MCLK, divides to BCLK, clk_out = BCLK */
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

    /* Pre-fill with silence (clocks still toggle correctly) */
    for (size_t i = 0; i < handle->dma_buf_count; i++) {
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
        xSemaphoreTake(handle->write_sem, 0);
    }

    const char *mode_str =
        (handle->mode == PARLIO_I2S_MODE_STANDARD) ? "I2S" :
        (handle->mode == PARLIO_I2S_MODE_TDM4) ? "TDM4" :
        (handle->mode == PARLIO_I2S_MODE_TDM8) ? "TDM8" : "TDM16";
    ESP_LOGI(TAG, "TX enabled: %s, Fs=%"PRIu32", %u-bit, %u ch (%u lines x %u slots)",
             mode_str, handle->sample_rate, handle->bits_per_sample,
             handle->num_slots * handle->num_data_lines,
             handle->num_data_lines, handle->num_slots);
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

    const size_t channels_per_frame = handle->num_data_lines * handle->num_slots;
    size_t written = 0;

    while (written < num_frames) {
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

            if (xSemaphoreTake(handle->write_sem,
                               pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
                if (frames_written) *frames_written = written;
                return ESP_ERR_TIMEOUT;
            }
        }

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
    return handle->sample_rate;
}
