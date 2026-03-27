#include "parlio_adat_tx.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/parlio_tx.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "parlio_adat";

/*
 * ADAT Lightpipe frame format:
 *
 * Each frame = 256 bit-cells at the NRZI bit rate (256 * Fs).
 *
 * Structure:
 *   - Sync: 1 followed by 9 zeros (10 bit-cells)
 *   - 8 channels, each 30 bit-cells:
 *     - 1 separator bit (always 1)
 *     - For each of 6 nibbles (24 bits total, MSB first):
 *       - 4 data bits
 *       - 1 separator bit (always 1) -- except after the last nibble
 *     Total per channel: 1 + 24 + 5 = 30 bit-cells
 *   - Remaining: 8*30 + 10 = 250, pad to 256 with 6 bits
 *
 * NRZI encoding: a '1' in the pre-NRZI data causes a level transition
 * on the output. A '0' maintains the current level.
 *
 * The output alternates between high and low on each '1' bit,
 * allowing the receiver to recover the clock from transitions.
 */

#define ADAT_BITS_PER_FRAME  256
#define ADAT_BYTES_PER_FRAME (ADAT_BITS_PER_FRAME / 8)  /* 32 bytes */
#define ADAT_CHANNELS        8

struct parlio_adat_tx {
    uint32_t sample_rate;
    uint32_t adat_clock;        /* 256 * Fs */

    /* NRZI encoder state */
    uint8_t nrzi_state;

    /* DMA buffers */
    size_t   frame_buf_bytes;
    size_t   dma_buf_bytes;
    size_t   frames_per_buf;
    size_t   dma_buf_count;
    uint8_t  **dma_bufs;
    size_t   write_buf_idx;
    size_t   write_frame_pos;

    SemaphoreHandle_t write_sem;

    i2s_chan_handle_t       i2s_clk_chan;
    parlio_tx_unit_handle_t parlio_unit;
    bool enabled;
};

/* Write one NRZI-encoded bit to the output buffer.
 * A '1' pre-NRZI bit causes a transition, '0' holds the level. */
static inline void nrzi_encode_bit(uint8_t *buf, size_t *bit_idx,
                                    uint8_t bit, uint8_t *state)
{
    if (bit) *state ^= 1;
    size_t idx = *bit_idx;
    if (*state) {
        buf[idx / 8] |= (1 << (idx % 8));
    }
    (*bit_idx)++;
}

/*
 * Encode one ADAT frame (8 channels of 24-bit audio) into NRZI format.
 * Output: 256 bits = 32 bytes.
 *
 * samples[0..7] = channels 0-7 as left-justified int32_t (upper 24 bits used).
 */
static void encode_adat_frame(struct parlio_adat_tx *h,
                               uint8_t *dst,
                               const int32_t *samples)
{
    memset(dst, 0, ADAT_BYTES_PER_FRAME);
    size_t bi = 0;

    /* Sync pattern: 1 followed by 9 zeros.
     * In NRZI: the '1' causes a transition, then 9 zeros hold the level.
     * This creates a long run without transitions, which the receiver
     * uses for frame synchronization. */
    nrzi_encode_bit(dst, &bi, 1, &h->nrzi_state);
    for (int i = 0; i < 9; i++) {
        nrzi_encode_bit(dst, &bi, 0, &h->nrzi_state);
    }

    /* 8 channels */
    for (int ch = 0; ch < ADAT_CHANNELS; ch++) {
        /* Extract 24 bits from left-justified int32_t */
        uint32_t audio = 0;
        if (samples) {
            audio = ((uint32_t)samples[ch]) >> 8;
        }

        /* Channel separator (always 1 -> causes a transition in NRZI) */
        nrzi_encode_bit(dst, &bi, 1, &h->nrzi_state);

        /* 24 data bits, MSB first, with a separator '1' after every 4 bits */
        for (int nibble = 0; nibble < 6; nibble++) {
            /* 4 data bits from the current nibble (MSB first) */
            int shift = 20 - nibble * 4; /* start from bit 23 (MSB) */
            for (int b = 3; b >= 0; b--) {
                uint8_t bit = (audio >> (shift + b)) & 1;
                nrzi_encode_bit(dst, &bi, bit, &h->nrzi_state);
            }

            /* Separator after each nibble (except the last one is the
             * channel separator of the next channel, or padding) */
            if (nibble < 5) {
                nrzi_encode_bit(dst, &bi, 1, &h->nrzi_state);
            }
        }
    }

    /* Pad remaining bits to 256 with zeros (no transitions) */
    while (bi < ADAT_BITS_PER_FRAME) {
        nrzi_encode_bit(dst, &bi, 0, &h->nrzi_state);
    }
}

static bool IRAM_ATTR on_tx_done(parlio_tx_unit_handle_t tx_unit,
                                  const parlio_tx_done_event_data_t *edata,
                                  void *ctx)
{
    struct parlio_adat_tx *h = (struct parlio_adat_tx *)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(h->write_sem, &woken);
    return woken == pdTRUE;
}

/* --- public API --- */

esp_err_t parlio_adat_tx_new(const parlio_adat_tx_config_t *config,
                              parlio_adat_tx_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "null arg");

    uint16_t mclk_mult = config->mclk_multiple ? config->mclk_multiple : 512;
    ESP_RETURN_ON_FALSE(mclk_mult >= 256 && (mclk_mult % 256) == 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "mclk_multiple must be >= 256 and a multiple of 256");

    struct parlio_adat_tx *h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "alloc handle");
    esp_err_t ret = ESP_OK;

    h->sample_rate     = config->sample_rate;
    h->adat_clock      = 256 * config->sample_rate;
    h->frame_buf_bytes = ADAT_BYTES_PER_FRAME;
    h->frames_per_buf  = config->frames_per_buffer ? config->frames_per_buffer : 64;
    h->dma_buf_count   = config->dma_buffer_count  ? config->dma_buffer_count  : 4;
    h->dma_buf_bytes   = h->frame_buf_bytes * h->frames_per_buf;

    ESP_LOGI(TAG, "ADAT: Fs=%"PRIu32", 8ch x 24-bit, clock=%"PRIu32" Hz, "
             "frame=%u B, dma=%u B x %u",
             h->sample_rate, h->adat_clock,
             (unsigned)h->frame_buf_bytes, (unsigned)h->dma_buf_bytes,
             (unsigned)h->dma_buf_count);

    /* Allocate DMA buffers */
    h->dma_bufs = calloc(h->dma_buf_count, sizeof(uint8_t *));
    ESP_GOTO_ON_FALSE(h->dma_bufs, ESP_ERR_NO_MEM, fail, TAG, "alloc buf array");
    for (size_t i = 0; i < h->dma_buf_count; i++) {
        h->dma_bufs[i] = heap_caps_calloc(1, h->dma_buf_bytes,
                                           MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        ESP_GOTO_ON_FALSE(h->dma_bufs[i], ESP_ERR_NO_MEM, fail, TAG, "alloc DMA buf");
    }

    h->write_sem = xSemaphoreCreateCounting(h->dma_buf_count, h->dma_buf_count);
    ESP_GOTO_ON_FALSE(h->write_sem, ESP_ERR_NO_MEM, fail, TAG, "alloc semaphore");

    /* I2S MCLK from APLL */
    {
        i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
        cc.dma_desc_num = 2; cc.dma_frame_num = 16;
        ret = i2s_new_channel(&cc, &h->i2s_clk_chan, NULL);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "I2S channel alloc failed");

        i2s_std_config_t sc = {
            .clk_cfg = { .sample_rate_hz = config->sample_rate,
                         .clk_src = I2S_CLK_SRC_APLL,
                         .mclk_multiple = (i2s_mclk_multiple_t)mclk_mult },
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                             I2S_SLOT_MODE_STEREO),
            .gpio_cfg = { .mclk = config->mclk_gpio,
                          .bclk = I2S_GPIO_UNUSED, .ws = I2S_GPIO_UNUSED,
                          .dout = I2S_GPIO_UNUSED, .din = I2S_GPIO_UNUSED },
        };
        ret = i2s_channel_init_std_mode(h->i2s_clk_chan, &sc);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "I2S init failed");
        ret = i2s_channel_enable(h->i2s_clk_chan);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "I2S enable failed");
    }

    /* PARLIO TX: 1-bit serial output at 256 * Fs */
    {
        uint32_t mclk_freq = (uint32_t)mclk_mult * config->sample_rate;
        parlio_tx_unit_config_t pc = {
            .clk_src = PARLIO_CLK_SRC_DEFAULT,
            .clk_in_gpio_num = config->mclk_gpio,
            .input_clk_src_freq_hz = mclk_freq,
            .output_clk_freq_hz = h->adat_clock,
            .data_width = 1,
            .clk_out_gpio_num = -1,
            .valid_gpio_num = -1,
            .trans_queue_depth = h->dma_buf_count,
            .max_transfer_size = h->dma_buf_bytes,
            .sample_edge = PARLIO_SAMPLE_EDGE_POS,
            .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
            .flags = { .clk_gate_en = 0, .io_loop_back = 0 },
        };
        for (int i = 0; i < 16; i++) pc.data_gpio_nums[i] = -1;
        pc.data_gpio_nums[0] = config->adat_gpio;

        ret = parlio_new_tx_unit(&pc, &h->parlio_unit);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "PARLIO create failed");

        parlio_tx_event_callbacks_t cbs = { .on_trans_done = on_tx_done };
        ret = parlio_tx_unit_register_event_callbacks(h->parlio_unit, &cbs, h);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "PARLIO callbacks failed");
    }

    *ret_handle = h;
    return ESP_OK;

fail:
    parlio_adat_tx_delete(h);
    return ESP_FAIL;
}

esp_err_t parlio_adat_tx_enable(parlio_adat_tx_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle && !handle->enabled, ESP_ERR_INVALID_STATE, TAG, "bad state");
    ESP_RETURN_ON_ERROR(parlio_tx_unit_enable(handle->parlio_unit), TAG, "enable failed");

    handle->enabled = true;
    handle->write_buf_idx = 0;
    handle->write_frame_pos = 0;
    handle->nrzi_state = 0;

    /* Pre-fill with silence */
    for (size_t i = 0; i < handle->dma_buf_count; i++) {
        for (size_t f = 0; f < handle->frames_per_buf; f++) {
            uint8_t *dst = handle->dma_bufs[i] + f * handle->frame_buf_bytes;
            encode_adat_frame(handle, dst, NULL);
        }
        parlio_transmit_config_t tc = { .idle_value = 0 };
        ESP_RETURN_ON_ERROR(
            parlio_tx_unit_transmit(handle->parlio_unit, handle->dma_bufs[i],
                                    handle->dma_buf_bytes * 8, &tc),
            TAG, "silence transmit failed");
        xSemaphoreTake(handle->write_sem, 0);
    }

    ESP_LOGI(TAG, "ADAT TX enabled: Fs=%"PRIu32", 8 channels", handle->sample_rate);
    return ESP_OK;
}

esp_err_t parlio_adat_tx_disable(parlio_adat_tx_handle_t handle)
{
    if (!handle || !handle->enabled) return ESP_OK;
    parlio_tx_unit_wait_all_done(handle->parlio_unit, 500);
    parlio_tx_unit_disable(handle->parlio_unit);
    handle->enabled = false;
    return ESP_OK;
}

esp_err_t parlio_adat_tx_write(parlio_adat_tx_handle_t handle,
                                const int32_t *samples,
                                size_t num_frames,
                                size_t *frames_written,
                                uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && samples && handle->enabled,
                        ESP_ERR_INVALID_ARG, TAG, "bad arg/state");

    size_t written = 0;
    while (written < num_frames) {
        if (handle->write_frame_pos >= handle->frames_per_buf) {
            parlio_transmit_config_t tc = { .idle_value = 0 };
            esp_err_t ret = parlio_tx_unit_transmit(
                handle->parlio_unit, handle->dma_bufs[handle->write_buf_idx],
                handle->dma_buf_bytes * 8, &tc);
            ESP_RETURN_ON_ERROR(ret, TAG, "transmit failed");

            handle->write_buf_idx = (handle->write_buf_idx + 1) % handle->dma_buf_count;
            handle->write_frame_pos = 0;

            if (xSemaphoreTake(handle->write_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
                if (frames_written) *frames_written = written;
                return ESP_ERR_TIMEOUT;
            }
        }

        uint8_t *dst = handle->dma_bufs[handle->write_buf_idx]
                     + handle->write_frame_pos * handle->frame_buf_bytes;
        encode_adat_frame(handle, dst, &samples[written * ADAT_CHANNELS]);
        handle->write_frame_pos++;
        written++;
    }

    if (frames_written) *frames_written = written;
    return ESP_OK;
}

esp_err_t parlio_adat_tx_delete(parlio_adat_tx_handle_t handle)
{
    if (!handle) return ESP_OK;
    if (handle->enabled) parlio_adat_tx_disable(handle);
    if (handle->parlio_unit) parlio_del_tx_unit(handle->parlio_unit);
    if (handle->i2s_clk_chan) {
        i2s_channel_disable(handle->i2s_clk_chan);
        i2s_del_channel(handle->i2s_clk_chan);
    }
    if (handle->dma_bufs) {
        for (size_t i = 0; i < handle->dma_buf_count; i++) free(handle->dma_bufs[i]);
        free(handle->dma_bufs);
    }
    if (handle->write_sem) vSemaphoreDelete(handle->write_sem);
    free(handle);
    return ESP_OK;
}
