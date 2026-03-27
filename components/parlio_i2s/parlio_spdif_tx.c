#include "parlio_spdif_tx.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/parlio_tx.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "parlio_spdif";

/*
 * IEC60958 / S/PDIF frame structure:
 *
 * Each frame = 2 subframes (left + right), each subframe = 32 bits:
 *   [3:0]   preamble (4 symbols in BMC, special violation patterns)
 *   [7:4]   auxiliary / LSB audio (4 bits)
 *   [27:8]  audio sample (20 bits, or up to 24 if aux is used for audio)
 *   [28]    validity bit (0 = valid)
 *   [29]    user data bit
 *   [30]    channel status bit
 *   [31]    parity bit (even parity over bits 4-31)
 *
 * Biphase mark coding (BMC):
 *   - Transition at the start of every bit cell
 *   - Additional transition in the middle for a '1' bit
 *   - Each bit = 2 UI (unit intervals)
 *   - So each 32-bit subframe = 64 UI
 *   - Each frame = 128 UI -> PARLIO clock = 128 * Fs
 *
 * Preambles (violate BMC rules for sync, encoded as 4-symbol sequences):
 *   B (block start, left):  11101000 or 00010111
 *   M (left, not block start): 11100010 or 00011101
 *   W (right):              11100100 or 00011011
 * The choice depends on the running parity of the BMC encoder.
 */

/* Preamble patterns: [inverted=0/1][type B/M/W] -> 8 UI (4 bit-cells worth) */
static const uint8_t PREAMBLE_B[2] = { 0xE8, 0x17 }; /* block start, left ch */
static const uint8_t PREAMBLE_M[2] = { 0xE2, 0x1D }; /* left ch, not block start */
static const uint8_t PREAMBLE_W[2] = { 0xE4, 0x1B }; /* right ch */

/* Channel status bits for consumer format, 48 kHz, 2ch, no copy protection.
 * 192 bits (24 bytes), transmitted LSB first within each byte. */
static const uint8_t CONSUMER_CHANNEL_STATUS[24] = {
    0x04, /* byte 0: consumer, PCM, no copy, 2ch */
    0x00, /* byte 1: category code (general) */
    0x00, /* byte 2: source/channel number */
    0x02, /* byte 3: Fs = 48 kHz (bits 24-27 = 0010) */
    0x0B, /* byte 4: max 24-bit word length, sample = 24 bit */
    /* bytes 5-23: reserved, zero */
};

/* Professional format channel status for 48 kHz */
static const uint8_t PROFESSIONAL_CHANNEL_STATUS[24] = {
    0x01, /* byte 0: professional, non-audio=0, emphasis=0 */
    0x00, /* byte 1 */
    0x00, /* byte 2 */
    0x02, /* byte 3: Fs = 48 kHz */
    0x00, /* byte 4 */
};

struct parlio_spdif_tx {
    uint32_t sample_rate;
    uint8_t  bits_per_sample;
    uint32_t spdif_clock;       /* 128 * Fs */

    /* channel status block (192 bits = 24 bytes) */
    uint8_t  channel_status[24];
    uint16_t frame_in_block;    /* 0..191, position within 192-frame block */

    /* BMC encoder state: 0 or 1 (running level of the output line) */
    uint8_t  bmc_state;

    /* DMA buffer management */
    size_t   frame_buf_bytes;   /* bytes per audio frame in BMC format (128 bits / 8 = 16) */
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

/* Encode one bit in BMC, appending 2 UI to the buffer.
 * Returns the new BMC state. */
static inline uint8_t bmc_encode_bit(uint8_t *buf, size_t *ui_idx,
                                      uint8_t bit, uint8_t state)
{
    /* Transition at start of every bit cell */
    state ^= 1;
    size_t idx = *ui_idx;
    buf[idx / 8] |= (state << (idx % 8));
    idx++;

    /* Additional transition in the middle for a '1' bit */
    if (bit) state ^= 1;
    buf[idx / 8] |= (state << (idx % 8));
    idx++;

    *ui_idx = idx;
    return state;
}

/* Encode a preamble (4 bit-cells = 8 UI), which violates normal BMC rules. */
static inline uint8_t bmc_encode_preamble(uint8_t *buf, size_t *ui_idx,
                                           const uint8_t preamble[2],
                                           uint8_t state)
{
    uint8_t pattern = preamble[state & 1];
    size_t idx = *ui_idx;
    for (int i = 0; i < 8; i++) {
        uint8_t val = (pattern >> i) & 1;
        buf[idx / 8] |= (val << (idx % 8));
        idx++;
    }
    /* Update BMC state based on last UI of preamble */
    state = (pattern >> 7) & 1;
    *ui_idx = idx;
    return state;
}

/*
 * Encode one S/PDIF frame (left + right subframes) into BMC format.
 * Output: 128 UI = 16 bytes.
 */
static void encode_spdif_frame(struct parlio_spdif_tx *h,
                                uint8_t *dst,
                                int32_t left, int32_t right)
{
    memset(dst, 0, 16);
    size_t ui = 0;

    for (int sub = 0; sub < 2; sub++) {
        /* Select preamble */
        const uint8_t *preamble;
        if (sub == 0) {
            preamble = (h->frame_in_block == 0) ? PREAMBLE_B : PREAMBLE_M;
        } else {
            preamble = PREAMBLE_W;
        }
        h->bmc_state = bmc_encode_preamble(dst, &ui, preamble, h->bmc_state);

        /* Get the sample for this subframe */
        int32_t samp = (sub == 0) ? left : right;
        /* Shift to get the audio bits in the right position.
         * S/PDIF subframe bits 4-27 carry audio (24 bits, MSB at bit 27).
         * Our samples are left-justified int32_t, so shift right by 8 to
         * get 24 bits, then we transmit LSB first (bit 4 first). */
        uint32_t audio = (uint32_t)samp >> 8; /* upper 24 bits */

        /* Encode bits 4-27: audio data, LSB first */
        /* First 4 bits (aux/LSB, bits 4-7): lower nibble of audio */
        uint32_t subframe_data = audio & 0x00FFFFFF;
        uint8_t parity = 0;
        for (int bit = 0; bit < 24; bit++) {
            uint8_t b = (subframe_data >> bit) & 1;
            h->bmc_state = bmc_encode_bit(dst, &ui, b, h->bmc_state);
            parity ^= b;
        }

        /* Bit 28: validity (0 = valid audio) */
        h->bmc_state = bmc_encode_bit(dst, &ui, 0, h->bmc_state);

        /* Bit 29: user data (0) */
        h->bmc_state = bmc_encode_bit(dst, &ui, 0, h->bmc_state);

        /* Bit 30: channel status */
        uint8_t cs_bit = 0;
        if (h->frame_in_block < 192) {
            uint8_t byte_idx = h->frame_in_block / 8;
            uint8_t bit_idx  = h->frame_in_block % 8;
            if (byte_idx < 24) {
                cs_bit = (h->channel_status[byte_idx] >> bit_idx) & 1;
            }
        }
        h->bmc_state = bmc_encode_bit(dst, &ui, cs_bit, h->bmc_state);
        parity ^= cs_bit;

        /* Bit 31: parity (even parity over bits 4-31) */
        h->bmc_state = bmc_encode_bit(dst, &ui, parity, h->bmc_state);
    }

    /* Advance the block position */
    h->frame_in_block++;
    if (h->frame_in_block >= 192) {
        h->frame_in_block = 0;
    }
}

static bool IRAM_ATTR on_tx_done(parlio_tx_unit_handle_t tx_unit,
                                  const parlio_tx_done_event_data_t *edata,
                                  void *ctx)
{
    struct parlio_spdif_tx *h = (struct parlio_spdif_tx *)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(h->write_sem, &woken);
    return woken == pdTRUE;
}

/* --- public API --- */

esp_err_t parlio_spdif_tx_new(const parlio_spdif_tx_config_t *config,
                               parlio_spdif_tx_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(config->bits_per_sample == 16 ||
                        config->bits_per_sample == 20 ||
                        config->bits_per_sample == 24,
                        ESP_ERR_INVALID_ARG, TAG, "unsupported bits_per_sample");

    uint16_t mclk_mult = config->mclk_multiple ? config->mclk_multiple : 256;
    ESP_RETURN_ON_FALSE(mclk_mult >= 128 && (mclk_mult % 128) == 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "mclk_multiple must be >= 128 and a multiple of 128");

    struct parlio_spdif_tx *h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "alloc handle");
    esp_err_t ret = ESP_OK;

    h->sample_rate     = config->sample_rate;
    h->bits_per_sample = config->bits_per_sample;
    h->spdif_clock     = 128 * config->sample_rate;

    /* Set up channel status */
    memcpy(h->channel_status,
           config->consumer_format ? CONSUMER_CHANNEL_STATUS : PROFESSIONAL_CHANNEL_STATUS,
           24);

    /* DMA sizing: 128 UI per frame, 1 bit per UI, packed into bytes = 16 bytes/frame */
    h->frame_buf_bytes = 16;
    h->frames_per_buf  = config->frames_per_buffer ? config->frames_per_buffer : 192;
    h->dma_buf_count   = config->dma_buffer_count  ? config->dma_buffer_count  : 4;
    h->dma_buf_bytes   = h->frame_buf_bytes * h->frames_per_buf;

    ESP_LOGI(TAG, "SPDIF: Fs=%"PRIu32", %u-bit, clock=%"PRIu32" Hz, "
             "frame=%u B, dma=%u B x %u",
             h->sample_rate, h->bits_per_sample, h->spdif_clock,
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

    /* PARLIO TX: 1-bit serial output at 128 * Fs */
    {
        uint32_t mclk_freq = (uint32_t)mclk_mult * config->sample_rate;
        parlio_tx_unit_config_t pc = {
            .clk_src = PARLIO_CLK_SRC_DEFAULT,
            .clk_in_gpio_num = config->mclk_gpio,
            .input_clk_src_freq_hz = mclk_freq,
            .output_clk_freq_hz = h->spdif_clock,
            .data_width = 1,
            .clk_out_gpio_num = -1,     /* no clock output needed */
            .valid_gpio_num = -1,
            .trans_queue_depth = h->dma_buf_count,
            .max_transfer_size = h->dma_buf_bytes,
            .sample_edge = PARLIO_SAMPLE_EDGE_POS,
            .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
            .flags = { .clk_gate_en = 0, .io_loop_back = 0 },
        };
        for (int i = 0; i < 16; i++) pc.data_gpio_nums[i] = -1;
        pc.data_gpio_nums[0] = config->spdif_gpio;

        ret = parlio_new_tx_unit(&pc, &h->parlio_unit);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "PARLIO create failed");

        parlio_tx_event_callbacks_t cbs = { .on_trans_done = on_tx_done };
        ret = parlio_tx_unit_register_event_callbacks(h->parlio_unit, &cbs, h);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "PARLIO callbacks failed");
    }

    *ret_handle = h;
    return ESP_OK;

fail:
    parlio_spdif_tx_delete(h);
    return ESP_FAIL;
}

esp_err_t parlio_spdif_tx_enable(parlio_spdif_tx_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle && !handle->enabled, ESP_ERR_INVALID_STATE, TAG, "bad state");
    ESP_RETURN_ON_ERROR(parlio_tx_unit_enable(handle->parlio_unit), TAG, "enable failed");

    handle->enabled = true;
    handle->write_buf_idx = 0;
    handle->write_frame_pos = 0;
    handle->frame_in_block = 0;
    handle->bmc_state = 0;

    /* Pre-fill with silence */
    for (size_t i = 0; i < handle->dma_buf_count; i++) {
        for (size_t f = 0; f < handle->frames_per_buf; f++) {
            uint8_t *dst = handle->dma_bufs[i] + f * handle->frame_buf_bytes;
            encode_spdif_frame(handle, dst, 0, 0);
        }
        parlio_transmit_config_t tc = { .idle_value = 0 };
        ESP_RETURN_ON_ERROR(
            parlio_tx_unit_transmit(handle->parlio_unit, handle->dma_bufs[i],
                                    handle->dma_buf_bytes * 8, &tc),
            TAG, "silence transmit failed");
        xSemaphoreTake(handle->write_sem, 0);
    }

    ESP_LOGI(TAG, "SPDIF TX enabled: Fs=%"PRIu32, handle->sample_rate);
    return ESP_OK;
}

esp_err_t parlio_spdif_tx_disable(parlio_spdif_tx_handle_t handle)
{
    if (!handle || !handle->enabled) return ESP_OK;
    parlio_tx_unit_wait_all_done(handle->parlio_unit, 500);
    parlio_tx_unit_disable(handle->parlio_unit);
    handle->enabled = false;
    return ESP_OK;
}

esp_err_t parlio_spdif_tx_write(parlio_spdif_tx_handle_t handle,
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
        encode_spdif_frame(handle, dst,
                          samples[written * 2], samples[written * 2 + 1]);
        handle->write_frame_pos++;
        written++;
    }

    if (frames_written) *frames_written = written;
    return ESP_OK;
}

esp_err_t parlio_spdif_tx_delete(parlio_spdif_tx_handle_t handle)
{
    if (!handle) return ESP_OK;
    if (handle->enabled) parlio_spdif_tx_disable(handle);
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
