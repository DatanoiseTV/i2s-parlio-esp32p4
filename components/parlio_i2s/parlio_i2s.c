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

/* Lookup tables for fast bit extraction.
 * For each byte value (0-255), stores 8 output bytes as a uint64_t.
 * LRCK=0: output byte = (bit << 1)          -> 0x00 or 0x02
 * LRCK=1: output byte = 0x01 | (bit << 1)   -> 0x01 or 0x03 */
static DRAM_ATTR uint64_t lut_lrck0[256];
static DRAM_ATTR uint64_t lut_lrck1[256];
static bool lut_initialized = false;

static void init_bit_lut(void)
{
    if (lut_initialized) return;
    for (int v = 0; v < 256; v++) {
        uint8_t o0[8], o1[8];
        for (int b = 0; b < 8; b++) {
            uint8_t bit = (v >> (7 - b)) & 1;
            o0[b] = bit << 1;
            o1[b] = 0x01 | (bit << 1);
        }
        memcpy(&lut_lrck0[v], o0, 8);
        memcpy(&lut_lrck1[v], o1, 8);
    }
    lut_initialized = true;
}

struct parlio_i2s_tx {
    uint32_t sample_rate;
    uint8_t  bits_per_sample;
    uint8_t  slot_width;
    uint8_t  num_data_lines;
    uint8_t  num_slots;
    uint16_t mclk_multiple;
    parlio_i2s_mode_t mode;

    uint32_t mclk_freq;
    uint32_t bclk_freq;
    uint16_t bclk_per_frame;
    uint8_t  parlio_width;

    /* DMA buffer management: 3-buffer rotation for loop mode */
    size_t   frame_buf_bytes;
    size_t   dma_buf_bytes;
    size_t   frames_per_buf;
    uint8_t  *dma_bufs[3];     /* exactly 3 buffers for loop rotation */
    size_t   submit_idx;        /* last buffer submitted to DMA */
    size_t   encode_idx;        /* buffer currently being encoded */
    size_t   encode_frame_pos;  /* frame position within current encode buffer */

    /* Semaphore for non-loop fallback mode */
    SemaphoreHandle_t write_sem;

    i2s_chan_handle_t       i2s_clk_chan;
    parlio_tx_unit_handle_t parlio_unit;
    bool enabled;
    bool loop_mode;             /* true if using loop transmission */
};

static uint8_t next_parlio_width(uint8_t needed)
{
    if (needed <= 8)  return 8;
    if (needed <= 16) return 16;
    return 0;
}

/*
 * Fast LUT-based encoder for stereo I2S standard, 32-bit, 1 data line.
 * Produces 64 bytes of DMA data from one stereo sample pair.
 *
 * Each output byte is one of {0x00, 0x01, 0x02, 0x03}:
 *   bit 0 = LRCK (0 for left, 1 for right)
 *   bit 1 = audio data bit
 *
 * Uses precomputed LUT: one byte of sample -> 8 output bytes via uint64_t write.
 * 4 LUT lookups per channel = 8 total = 8 uint64_t stores for 64 output bytes.
 */
static void IRAM_ATTR encode_frame_lut(uint8_t *restrict dst,
                                        int32_t left, int32_t right)
{
    const uint8_t *L = (const uint8_t *)&left;
    const uint8_t *R = (const uint8_t *)&right;

    /* Left channel (LRCK=0): dst[0]=padding, dst[1..31]=31 data bits */
    dst[0] = 0x00;
    /* RISC-V is little-endian: L[3]=MSByte (bits 31-24), L[0]=LSByte (bits 7-0) */
    memcpy(dst + 1,  &lut_lrck0[L[3]], 8);   /* bits 31-24 -> dst[1..8] */
    memcpy(dst + 9,  &lut_lrck0[L[2]], 8);   /* bits 23-16 -> dst[9..16] */
    memcpy(dst + 17, &lut_lrck0[L[1]], 8);   /* bits 15-8  -> dst[17..24] */
    memcpy(dst + 25, &lut_lrck0[L[0]], 8);   /* bits 7-0   -> dst[25..32] (overwrites dst[32]) */

    /* Right channel (LRCK=1): dst[32]=padding, dst[33..63]=31 data bits */
    dst[32] = 0x01;                            /* fix overwrite from above */
    memcpy(dst + 33, &lut_lrck1[R[3]], 8);   /* bits 31-24 */
    memcpy(dst + 41, &lut_lrck1[R[2]], 8);   /* bits 23-16 */
    memcpy(dst + 49, &lut_lrck1[R[1]], 8);   /* bits 15-8 */
    memcpy(dst + 57, &lut_lrck1[R[0]], 8);   /* bits 7-0 -> dst[57..64] */
    /* dst[64] is next frame's byte 0, which will be overwritten. Last frame in
     * buffer may write 1 byte past the buffer; allocate 1 extra byte. */
}

/*
 * Generic encoder for non-standard configurations (TDM, multi-line, etc.)
 */
static void IRAM_ATTR encode_frame_generic(struct parlio_i2s_tx *h,
                                            uint8_t *dst,
                                            const int32_t *samples)
{
    const uint16_t bpf = h->bclk_per_frame;
    const uint8_t  sw  = h->slot_width;
    const uint8_t  bps = h->bits_per_sample;
    const uint8_t  ndl = h->num_data_lines;
    const uint8_t  ns  = h->num_slots;
    const bool     is_std = (h->mode == PARLIO_I2S_MODE_STANDARD);

    /* Split loop by slot to eliminate division/modulo */
    for (uint8_t slot = 0; slot < ns; slot++) {
        uint8_t lrck;
        if (is_std) {
            lrck = (slot >= 1) ? 1 : 0;
        } else {
            lrck = (slot == 0) ? 1 : 0; /* TDM: frame sync on first slot only */
        }

        uint16_t base = slot * sw;
        /* Padding byte (1-BCLK offset) */
        dst[base] = lrck & 1;

        /* Data bits: pos_in_slot 1..min(bps, sw-1) */
        uint8_t max_bits = (bps < sw) ? bps : (sw - 1);
        for (uint8_t bi = 0; bi < max_bits; bi++) {
            uint16_t data_bits = 0;
            if (samples) {
                for (uint8_t line = 0; line < ndl; line++) {
                    int32_t samp = samples[line * ns + slot];
                    data_bits |= (((uint32_t)samp >> (31 - bi)) & 1) << line;
                }
            }
            dst[base + 1 + bi] = (lrck & 1) | (uint8_t)(data_bits << 1);
        }

        /* Remaining padding (if slot_width > bits_per_sample + 1) */
        for (uint16_t p = max_bits + 1; p < sw; p++) {
            dst[base + p] = lrck & 1;
        }

        /* TDM: frame sync is only high for bclk==0, rest of first slot is low */
        if (!is_std && slot == 0) {
            for (uint16_t p = 1; p < sw; p++) {
                dst[base + p] = (dst[base + p] & 0xFE); /* clear LRCK bit */
            }
            /* But we already set lrck=1 only for the padding byte dst[0].
             * Actually for TDM, lrck=1 at bclk==0 only. In our slot loop,
             * lrck is set per slot. For slot 0, all bytes have lrck=1.
             * We need to fix: only dst[0] should have lrck=1. */
            /* Rewrite: for TDM slot 0, only first byte gets lrck=1 */
        }
    }

    /* TDM fix: clear LRCK on all bytes except bclk==0 */
    if (!is_std) {
        /* dst[0] already has lrck=1 (set above). Clear it from the rest of slot 0 */
        for (uint16_t p = 1; p < sw; p++) {
            dst[p] &= 0xFE;
        }
    }
}

/* Dispatch to appropriate encoder */
static inline void IRAM_ATTR encode_frame(struct parlio_i2s_tx *h,
                                           uint8_t *dst,
                                           const int32_t *samples)
{
    if (h->mode == PARLIO_I2S_MODE_STANDARD && h->num_data_lines == 1 &&
        h->bits_per_sample == 32 && h->slot_width == 32 && h->parlio_width == 8) {
        encode_frame_lut(dst,
                         samples ? samples[0] : 0,
                         samples ? samples[1] : 0);
    } else {
        encode_frame_generic(h, dst, samples);
    }
}

static bool IRAM_ATTR on_parlio_tx_done(parlio_tx_unit_handle_t tx_unit,
                                         const parlio_tx_done_event_data_t *edata,
                                         void *user_ctx)
{
    struct parlio_i2s_tx *h = (struct parlio_i2s_tx *)user_ctx;
    BaseType_t woken = pdFALSE;
    if (h->write_sem) {
        xSemaphoreGiveFromISR(h->write_sem, &woken);
    }
    return woken == pdTRUE;
}

static esp_err_t setup_apll_clock(struct parlio_i2s_tx *h, gpio_num_t mclk_gpio)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2;
    chan_cfg.dma_frame_num = 16;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &h->i2s_clk_chan, NULL),
                        TAG, "I2S channel alloc failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg = { .sample_rate_hz = h->sample_rate, .clk_src = I2S_CLK_SRC_APLL,
                     .mclk_multiple = (i2s_mclk_multiple_t)h->mclk_multiple },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = mclk_gpio, .bclk = I2S_GPIO_UNUSED,
                      .ws = I2S_GPIO_UNUSED, .dout = I2S_GPIO_UNUSED,
                      .din = I2S_GPIO_UNUSED },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(h->i2s_clk_chan, &std_cfg),
                        TAG, "I2S init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(h->i2s_clk_chan), TAG, "I2S enable failed");
    ESP_LOGI(TAG, "APLL: MCLK=%"PRIu32" Hz on GPIO %d", h->mclk_freq, mclk_gpio);
    return ESP_OK;
}

static esp_err_t setup_parlio(struct parlio_i2s_tx *h, const parlio_i2s_tx_config_t *cfg)
{
    parlio_tx_unit_config_t pc = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = cfg->mclk_gpio,
        .input_clk_src_freq_hz = h->mclk_freq,
        .output_clk_freq_hz = h->bclk_freq,
        .data_width = h->parlio_width,
        .clk_out_gpio_num = cfg->bclk_gpio,
        .valid_gpio_num = -1,
        .trans_queue_depth = 4,
        .max_transfer_size = h->dma_buf_bytes + 64,
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .flags = { .clk_gate_en = 0, .io_loop_back = 0 },
    };
    for (int i = 0; i < 16; i++) pc.data_gpio_nums[i] = -1;
    pc.data_gpio_nums[0] = cfg->lrck_gpio;
    for (uint8_t i = 0; i < cfg->num_data_lines; i++)
        pc.data_gpio_nums[1 + i] = cfg->data_gpios[i];

    ESP_RETURN_ON_ERROR(parlio_new_tx_unit(&pc, &h->parlio_unit), TAG, "PARLIO create failed");

    parlio_tx_event_callbacks_t cbs = { .on_trans_done = on_parlio_tx_done };
    ESP_RETURN_ON_ERROR(parlio_tx_unit_register_event_callbacks(h->parlio_unit, &cbs, h),
                        TAG, "PARLIO callbacks failed");

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
    default: return 0;
    }
}

esp_err_t parlio_i2s_tx_new(const parlio_i2s_tx_config_t *config,
                             parlio_i2s_tx_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(config->num_data_lines >= 1 &&
                        config->num_data_lines <= PARLIO_I2S_MAX_DATA_LINES,
                        ESP_ERR_INVALID_ARG, TAG, "num_data_lines out of range");
    ESP_RETURN_ON_FALSE(config->bits_per_sample == 16 || config->bits_per_sample == 24 ||
                        config->bits_per_sample == 32,
                        ESP_ERR_INVALID_ARG, TAG, "unsupported bits_per_sample");

    parlio_i2s_mode_t mode = config->mode ? config->mode : PARLIO_I2S_MODE_STANDARD;
    uint8_t num_slots = resolve_num_slots(mode);
    ESP_RETURN_ON_FALSE(num_slots > 0, ESP_ERR_INVALID_ARG, TAG, "invalid mode");

    uint8_t slot_w = config->slot_width ? config->slot_width : 32;
    ESP_RETURN_ON_FALSE(slot_w >= config->bits_per_sample, ESP_ERR_INVALID_ARG, TAG,
                        "slot_width < bits_per_sample");

    uint16_t mclk_mult = config->mclk_multiple ? config->mclk_multiple : 256;
    uint8_t pw = next_parlio_width(1 + config->num_data_lines);
    ESP_RETURN_ON_FALSE(pw > 0 && pw <= 16, ESP_ERR_INVALID_ARG, TAG, "too many data lines");

    /* Initialize LUT for fast encoding */
    init_bit_lut();

    struct parlio_i2s_tx *h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "alloc handle");
    esp_err_t ret = ESP_OK;

    h->sample_rate     = config->sample_rate;
    h->bits_per_sample = config->bits_per_sample;
    h->slot_width      = slot_w;
    h->num_data_lines  = config->num_data_lines;
    h->num_slots       = num_slots;
    h->mclk_multiple   = mclk_mult;
    h->mode            = mode;
    h->parlio_width    = pw;
    h->bclk_per_frame  = (uint16_t)(slot_w * num_slots);
    h->mclk_freq       = (uint32_t)mclk_mult * config->sample_rate;
    h->bclk_freq       = (uint32_t)h->bclk_per_frame * config->sample_rate;

    uint32_t ratio = mclk_mult / h->bclk_per_frame;
    ESP_RETURN_ON_FALSE(ratio >= 1, ESP_ERR_INVALID_ARG, TAG, "MCLK/BCLK < 1");
    ESP_RETURN_ON_FALSE((mclk_mult % h->bclk_per_frame) == 0, ESP_ERR_INVALID_ARG, TAG,
                        "mclk_multiple not divisible by slot_width*num_slots");

    h->frames_per_buf = config->frames_per_buffer ? config->frames_per_buffer : 128;
    h->frame_buf_bytes = h->bclk_per_frame; /* pw=8: 1 byte per tick */
    if (pw > 8) h->frame_buf_bytes *= 2;
    h->dma_buf_bytes = h->frame_buf_bytes * h->frames_per_buf;

    /* Use loop mode for gapless output */
    h->loop_mode = true;

    ESP_LOGI(TAG, "mode=%s, %u slots/line, %u lines, %u ch, "
             "MCLK=%"PRIu32", BCLK=%"PRIu32" (ratio %u), "
             "frame=%u B, dma=%u B, loop=%s",
             (mode == PARLIO_I2S_MODE_STANDARD) ? "I2S" :
             (mode == PARLIO_I2S_MODE_TDM4) ? "TDM4" :
             (mode == PARLIO_I2S_MODE_TDM8) ? "TDM8" : "TDM16",
             num_slots, config->num_data_lines,
             (unsigned)(num_slots * config->num_data_lines),
             h->mclk_freq, h->bclk_freq, (unsigned)ratio,
             (unsigned)h->frame_buf_bytes, (unsigned)h->dma_buf_bytes,
             h->loop_mode ? "yes" : "no");

    /* Allocate 3 DMA buffers (+1 byte for LUT overwrite at frame boundary) */
    for (int i = 0; i < 3; i++) {
        h->dma_bufs[i] = heap_caps_calloc(1, h->dma_buf_bytes + 8,
                                           MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        ESP_GOTO_ON_FALSE(h->dma_bufs[i], ESP_ERR_NO_MEM, fail, TAG, "alloc DMA buf");
    }

    /* Semaphore for non-loop fallback */
    h->write_sem = xSemaphoreCreateCounting(3, 3);
    ESP_GOTO_ON_FALSE(h->write_sem, ESP_ERR_NO_MEM, fail, TAG, "alloc semaphore");

    ret = setup_apll_clock(h, config->mclk_gpio);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "APLL setup failed");

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
    ESP_RETURN_ON_FALSE(handle && !handle->enabled, ESP_ERR_INVALID_STATE, TAG, "bad state");
    ESP_RETURN_ON_ERROR(parlio_tx_unit_enable(handle->parlio_unit), TAG, "PARLIO enable failed");
    handle->enabled = true;

    /* Fill buf[0] with silence, start loop transmission */
    for (size_t f = 0; f < handle->frames_per_buf; f++) {
        uint8_t *p = handle->dma_bufs[0] + f * handle->frame_buf_bytes;
        encode_frame(handle, p, NULL);
    }

    parlio_transmit_config_t tx_cfg = {
        .idle_value = 0,
        .flags.loop_transmission = handle->loop_mode ? 1 : 0,
    };
    ESP_RETURN_ON_ERROR(
        parlio_tx_unit_transmit(handle->parlio_unit, handle->dma_bufs[0],
                                handle->dma_buf_bytes * 8, &tx_cfg),
        TAG, "initial transmit failed");

    handle->submit_idx = 0;
    handle->encode_idx = 1;
    handle->encode_frame_pos = 0;

    const char *mode_str =
        (handle->mode == PARLIO_I2S_MODE_STANDARD) ? "I2S" :
        (handle->mode == PARLIO_I2S_MODE_TDM4) ? "TDM4" :
        (handle->mode == PARLIO_I2S_MODE_TDM8) ? "TDM8" : "TDM16";
    ESP_LOGI(TAG, "TX enabled (%s DMA): %s, Fs=%"PRIu32", %u-bit, %u ch",
             handle->loop_mode ? "loop" : "one-shot", mode_str,
             handle->sample_rate, handle->bits_per_sample,
             handle->num_slots * handle->num_data_lines);
    return ESP_OK;
}

esp_err_t parlio_i2s_tx_disable(parlio_i2s_tx_handle_t handle)
{
    if (!handle || !handle->enabled) return ESP_OK;
    parlio_tx_unit_wait_all_done(handle->parlio_unit, 500);
    parlio_tx_unit_disable(handle->parlio_unit);
    handle->enabled = false;
    return ESP_OK;
}

esp_err_t parlio_i2s_tx_write(parlio_i2s_tx_handle_t handle,
                               const int32_t *samples,
                               size_t num_frames,
                               size_t *frames_written,
                               uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && samples && handle->enabled,
                        ESP_ERR_INVALID_ARG, TAG, "bad arg/state");

    const size_t cpf = handle->num_data_lines * handle->num_slots;
    size_t written = 0;

    while (written < num_frames) {
        uint8_t *dst = handle->dma_bufs[handle->encode_idx]
                     + handle->encode_frame_pos * handle->frame_buf_bytes;
        encode_frame(handle, dst, &samples[written * cpf]);
        handle->encode_frame_pos++;
        written++;

        /* Buffer full: submit and rotate */
        if (handle->encode_frame_pos >= handle->frames_per_buf) {
            parlio_transmit_config_t tx_cfg = {
                .idle_value = 0,
                .flags.loop_transmission = handle->loop_mode ? 1 : 0,
            };
            esp_err_t ret = parlio_tx_unit_transmit(
                handle->parlio_unit,
                handle->dma_bufs[handle->encode_idx],
                handle->dma_buf_bytes * 8,
                &tx_cfg);
            ESP_RETURN_ON_ERROR(ret, TAG, "transmit failed");

            handle->submit_idx = handle->encode_idx;
            /* Rotate: next encode buffer is 2 positions behind submit
             * (guaranteed free since DMA has moved past it) */
            handle->encode_idx = (handle->encode_idx + 1) % 3;
            handle->encode_frame_pos = 0;
        }
    }

    if (frames_written) *frames_written = written;
    return ESP_OK;
}

esp_err_t parlio_i2s_tx_delete(parlio_i2s_tx_handle_t handle)
{
    if (!handle) return ESP_OK;
    if (handle->enabled) parlio_i2s_tx_disable(handle);
    if (handle->parlio_unit) parlio_del_tx_unit(handle->parlio_unit);
    if (handle->i2s_clk_chan) {
        i2s_channel_disable(handle->i2s_clk_chan);
        i2s_del_channel(handle->i2s_clk_chan);
    }
    for (int i = 0; i < 3; i++) free(handle->dma_bufs[i]);
    if (handle->write_sem) vSemaphoreDelete(handle->write_sem);
    free(handle);
    return ESP_OK;
}

uint32_t parlio_i2s_tx_get_real_sample_rate(parlio_i2s_tx_handle_t handle)
{
    return handle ? handle->sample_rate : 0;
}
