#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PARLIO-based I2S/TDM transmitter for ESP32-P4.
 *
 * Uses the Parallel IO peripheral to synthesize I2S output signals (MCLK, BCLK,
 * LRCK/FSYNC, and a variable number of data lines). Clock is derived from the APLL
 * via the I2S peripheral's MCLK output, looped back as PARLIO external clock input.
 *
 * Signal mapping on PARLIO data bus:
 *   bit 0 = BCLK
 *   bit 1 = LRCK / frame sync
 *   bit 2..N = audio data lines
 *
 * Supported modes:
 *   STANDARD (I2S Philips) -- 2 slots per data line, 50/50 LRCK duty cycle
 *   TDM4  -- 4 slots per data line, short frame sync pulse
 *   TDM8  -- 8 slots per data line, short frame sync pulse
 *   TDM16 -- 16 slots per data line, short frame sync pulse
 *
 * MCLK is output on the PARLIO clock output pin (directly mirrors the ext clock).
 *
 * Maximum channel counts (with 14 data lines):
 *   STANDARD: 14 x  2 =  28 channels
 *   TDM4:     14 x  4 =  56 channels
 *   TDM8:     14 x  8 = 112 channels
 *   TDM16:    14 x 16 = 224 channels
 */

/* Maximum number of parallel data outputs. */
#define PARLIO_I2S_MAX_DATA_LINES  14

/* Slot mode / protocol selection. The numeric value equals the slot count. */
typedef enum {
    PARLIO_I2S_MODE_STANDARD = 2,   /* I2S Philips: 2 slots, 50/50 LRCK */
    PARLIO_I2S_MODE_TDM4     = 4,   /* TDM 4 slots, 1-BCLK frame sync pulse */
    PARLIO_I2S_MODE_TDM8     = 8,   /* TDM 8 slots, 1-BCLK frame sync pulse */
    PARLIO_I2S_MODE_TDM16    = 16,  /* TDM 16 slots, 1-BCLK frame sync pulse */
} parlio_i2s_mode_t;

typedef struct parlio_i2s_tx *parlio_i2s_tx_handle_t;

typedef struct {
    uint32_t sample_rate;         /* target sample rate in Hz (e.g. 48000) */
    uint8_t  bits_per_sample;     /* valid data bits: 16, 24, or 32 */
    uint8_t  slot_width;          /* bits per slot (>= bits_per_sample, typically 32) */
    uint8_t  num_data_lines;      /* parallel data outputs, 1..PARLIO_I2S_MAX_DATA_LINES */
    uint16_t mclk_multiple;       /* MCLK = mclk_multiple * sample_rate, typically 256 */

    parlio_i2s_mode_t mode;       /* STANDARD, TDM4, TDM8, or TDM16 (default: STANDARD) */

    /* GPIO assignments */
    gpio_num_t mclk_gpio;         /* MCLK output pin */
    gpio_num_t apll_feed_gpio;    /* GPIO used to route APLL clock into PARLIO (loopback) */
    gpio_num_t bclk_gpio;         /* BCLK output (PARLIO data bit 0) */
    gpio_num_t lrck_gpio;         /* LRCK / frame sync output (PARLIO data bit 1) */
    gpio_num_t data_gpios[PARLIO_I2S_MAX_DATA_LINES]; /* audio data pins */

    /* DMA / buffering */
    size_t dma_buffer_count;      /* number of DMA descriptors (default: 4) */
    size_t frames_per_buffer;     /* audio frames per DMA buffer (default: 128) */
} parlio_i2s_tx_config_t;

/**
 * Create and configure a PARLIO-based I2S/TDM transmitter.
 * Internally sets up APLL, I2S (clock-only), and PARLIO TX.
 */
esp_err_t parlio_i2s_tx_new(const parlio_i2s_tx_config_t *config,
                            parlio_i2s_tx_handle_t *ret_handle);

/**
 * Enable the transmitter. MCLK, BCLK, and LRCK/FSYNC begin toggling immediately
 * (with silence on data lines until audio is written).
 */
esp_err_t parlio_i2s_tx_enable(parlio_i2s_tx_handle_t handle);

/**
 * Disable the transmitter. All output pins go idle.
 */
esp_err_t parlio_i2s_tx_disable(parlio_i2s_tx_handle_t handle);

/**
 * Write interleaved audio samples.
 *
 * Each frame contains (num_data_lines * num_slots) samples as int32_t,
 * ordered by data line first, then slot:
 *   STANDARD: [line0_L, line0_R, line1_L, line1_R, ...]
 *   TDM:      [line0_slot0, line0_slot1, ..., line0_slotN,
 *              line1_slot0, line1_slot1, ..., line1_slotN, ...]
 *
 * Data is left-justified in int32_t regardless of bits_per_sample.
 *
 * Blocks until space is available or timeout expires.
 * Returns the number of frames actually written via *frames_written.
 */
esp_err_t parlio_i2s_tx_write(parlio_i2s_tx_handle_t handle,
                              const int32_t *samples,
                              size_t num_frames,
                              size_t *frames_written,
                              uint32_t timeout_ms);

/**
 * Delete the transmitter and release all resources.
 * Must be disabled first.
 */
esp_err_t parlio_i2s_tx_delete(parlio_i2s_tx_handle_t handle);

/**
 * Query the actual achieved sample rate after APLL configuration.
 */
uint32_t parlio_i2s_tx_get_real_sample_rate(parlio_i2s_tx_handle_t handle);

#ifdef __cplusplus
}
#endif
