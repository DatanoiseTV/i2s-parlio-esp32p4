#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Unified PARLIO audio transmitter for ESP32-P4.
 *
 * Outputs multiple audio protocols simultaneously on a single PARLIO TX unit.
 * All data lines shift in parallel from DMA buffers at a common clock rate
 * determined by the fastest configured protocol.
 *
 * Supported protocols (any combination):
 *   I2S/TDM  -- multi-channel parallel audio (BCLK + LRCK + N data lines)
 *   S/PDIF   -- stereo biphase mark coded serial (1 data line)
 *   ADAT     -- 8-channel NRZI encoded serial (1 data line)
 *
 * Clock rate selection:
 *   ADAT present:   PARLIO clock = 256 * Fs (12.288 MHz at 48 kHz)
 *   S/PDIF only:    PARLIO clock = 128 * Fs
 *   I2S only:       PARLIO clock = BCLK rate (BCLK on dedicated clk_out pin)
 *
 * When multiple protocols are active, BCLK is synthesized on a TXD data line
 * (not on clk_out). Slower protocols repeat their symbols to match the master
 * clock rate. clk_out outputs the PARLIO clock (usable as MCLK by external DACs).
 *
 * When only I2S is configured, BCLK uses the dedicated clk_out pin and PARLIO
 * runs at BCLK rate for maximum efficiency (same as the standalone parlio_i2s driver).
 *
 * Sample layout per frame in the write buffer:
 *   [I2S channels...] [S/PDIF L, R] [ADAT ch0..ch7]
 *   Only groups for enabled protocols are present.
 */

#define PARLIO_AUDIO_MAX_I2S_DATA_LINES  12

typedef struct parlio_audio_tx *parlio_audio_tx_handle_t;

/* I2S/TDM slot mode */
typedef enum {
    PARLIO_AUDIO_I2S_STANDARD = 2,
    PARLIO_AUDIO_I2S_TDM4     = 4,
    PARLIO_AUDIO_I2S_TDM8     = 8,
    PARLIO_AUDIO_I2S_TDM16    = 16,
} parlio_audio_i2s_mode_t;

/* Per-protocol sub-configs. Pass NULL to disable a protocol. */

typedef struct {
    parlio_audio_i2s_mode_t mode;       /* STANDARD, TDM4, TDM8, TDM16 */
    uint8_t  bits_per_sample;           /* 16, 24, or 32 */
    uint8_t  slot_width;                /* bits per slot (>= bits_per_sample, typically 32) */
    uint8_t  num_data_lines;            /* 1..PARLIO_AUDIO_MAX_I2S_DATA_LINES */
    gpio_num_t bclk_gpio;              /* BCLK output */
    gpio_num_t lrck_gpio;              /* LRCK / frame sync output */
    gpio_num_t data_gpios[PARLIO_AUDIO_MAX_I2S_DATA_LINES];
} parlio_audio_i2s_config_t;

typedef struct {
    uint8_t  bits_per_sample;           /* 16, 20, or 24 */
    bool     consumer_format;           /* true = consumer, false = professional */
    gpio_num_t spdif_gpio;
} parlio_audio_spdif_config_t;

typedef struct {
    gpio_num_t adat_gpio;
} parlio_audio_adat_config_t;

/* Top-level configuration */
typedef struct {
    uint32_t sample_rate;               /* target Fs in Hz (e.g. 48000) */
    uint16_t mclk_multiple;            /* MCLK/Fs for APLL (0 = auto-select minimum) */

    gpio_num_t mclk_gpio;              /* I2S MCLK output + PARLIO ext clk input */
    gpio_num_t clk_out_gpio;           /* PARLIO clk_out pin, -1 = unused.
                                         * In I2S-only mode this outputs BCLK.
                                         * In multi-protocol mode this outputs PARLIO clock (= MCLK). */

    /* Protocol sub-configs: non-NULL to enable */
    const parlio_audio_i2s_config_t   *i2s;
    const parlio_audio_spdif_config_t *spdif;
    const parlio_audio_adat_config_t  *adat;

    size_t dma_buffer_count;            /* default: 4 */
    size_t frames_per_buffer;           /* default: 64 */
} parlio_audio_tx_config_t;

esp_err_t parlio_audio_tx_new(const parlio_audio_tx_config_t *config,
                               parlio_audio_tx_handle_t *ret_handle);
esp_err_t parlio_audio_tx_enable(parlio_audio_tx_handle_t handle);
esp_err_t parlio_audio_tx_disable(parlio_audio_tx_handle_t handle);

/**
 * Write audio frames for all enabled protocols.
 *
 * Each frame contains samples for all enabled protocols, concatenated:
 *   I2S:   num_data_lines * num_slots samples (line-major: [line0_slot0..slotN, line1_slot0..])
 *   S/PDIF: 2 samples [left, right]
 *   ADAT:   8 samples [ch0..ch7]
 *
 * All samples are left-justified int32_t.
 * Use parlio_audio_tx_get_frame_size() to query the total samples per frame.
 */
esp_err_t parlio_audio_tx_write(parlio_audio_tx_handle_t handle,
                                 const int32_t *samples,
                                 size_t num_frames,
                                 size_t *frames_written,
                                 uint32_t timeout_ms);

esp_err_t parlio_audio_tx_delete(parlio_audio_tx_handle_t handle);

/* Query total int32_t samples per audio frame */
size_t parlio_audio_tx_get_frame_size(parlio_audio_tx_handle_t handle);

/* Query the PARLIO shift clock rate in Hz */
uint32_t parlio_audio_tx_get_parlio_clock(parlio_audio_tx_handle_t handle);

#ifdef __cplusplus
}
#endif
