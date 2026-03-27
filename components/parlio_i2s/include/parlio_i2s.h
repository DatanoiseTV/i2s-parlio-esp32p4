#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PARLIO-based I2S transmitter for ESP32-P4.
 *
 * Uses the Parallel IO peripheral to synthesize I2S output signals (MCLK, BCLK,
 * LRCK, and a variable number of data lines). Clock is derived from the APLL via
 * the I2S peripheral's MCLK output, looped back as PARLIO external clock input.
 *
 * Signal mapping on PARLIO data bus:
 *   bit 0 = BCLK
 *   bit 1 = LRCK (word select)
 *   bit 2..N = audio data lines (one per stereo pair: L on LRCK=0, R on LRCK=1)
 *
 * MCLK is output on the PARLIO clock output pin (directly mirrors the ext clock).
 */

/* Maximum number of parallel data outputs (each carries one stereo pair). */
#define PARLIO_I2S_MAX_DATA_LINES  14

typedef struct parlio_i2s_tx *parlio_i2s_tx_handle_t;

typedef struct {
    uint32_t sample_rate;         /* target sample rate in Hz (e.g. 48000) */
    uint8_t  bits_per_sample;     /* valid data bits: 16, 24, or 32 */
    uint8_t  slot_width;          /* bits per slot (>= bits_per_sample, typically 32) */
    uint8_t  num_data_lines;      /* parallel data outputs, 1..PARLIO_I2S_MAX_DATA_LINES */
    uint16_t mclk_multiple;       /* MCLK = mclk_multiple * sample_rate, typically 256 */

    /* GPIO assignments */
    gpio_num_t mclk_gpio;         /* MCLK output pin */
    gpio_num_t apll_feed_gpio;    /* GPIO used to route APLL clock into PARLIO (loopback) */
    gpio_num_t bclk_gpio;         /* BCLK output (directly driven via PARLIO data bit 0) */
    gpio_num_t lrck_gpio;         /* LRCK / WS output (PARLIO data bit 1) */
    gpio_num_t data_gpios[PARLIO_I2S_MAX_DATA_LINES]; /* audio data pins */

    /* DMA / buffering */
    size_t dma_buffer_count;      /* number of DMA descriptors (default: 4) */
    size_t frames_per_buffer;     /* audio frames per DMA buffer (default: 128) */
} parlio_i2s_tx_config_t;

/**
 * Create and configure a PARLIO-based I2S transmitter.
 * Internally sets up APLL, I2S (clock-only), and PARLIO TX.
 */
esp_err_t parlio_i2s_tx_new(const parlio_i2s_tx_config_t *config,
                            parlio_i2s_tx_handle_t *ret_handle);

/**
 * Enable the transmitter. MCLK, BCLK, and LRCK begin toggling immediately
 * (with silence on data lines until audio is written).
 */
esp_err_t parlio_i2s_tx_enable(parlio_i2s_tx_handle_t handle);

/**
 * Disable the transmitter. All output pins go idle.
 */
esp_err_t parlio_i2s_tx_disable(parlio_i2s_tx_handle_t handle);

/**
 * Write interleaved audio samples. Each frame contains (num_data_lines * 2) samples
 * in int32_t, ordered: [line0_L, line0_R, line1_L, line1_R, ...].
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
