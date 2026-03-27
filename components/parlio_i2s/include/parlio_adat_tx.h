#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ADAT Lightpipe transmitter using PARLIO on ESP32-P4.
 *
 * Outputs an ADAT (Alesis Digital Audio Tape) bitstream on a single GPIO pin
 * using NRZI encoding. Clocked by the APLL via I2S MCLK loopback.
 *
 * ADAT carries 8 channels of 24-bit audio at 48 kHz (or 4 channels at 96 kHz
 * in S/MUX mode, not implemented here).
 *
 * Frame format (256 bits):
 *   - Sync pattern: 1 followed by 9 zeros (10 bits)
 *   - For each of 8 channels:
 *     - 1 user/status bit
 *     - 24 data bits (MSB first)
 *     - 1 separator (always 1) after every 4 bits (NRZI nibble separator)
 *   - Total: 10 + 8 * (1 + 24 + 5 separators) = 10 + 8*30 = 250 bits
 *     + padding to 256
 *
 * NRZI encoding: a '1' bit causes a transition, a '0' bit holds the line.
 * PARLIO clock = 256 * Fs = 12.288 MHz at 48 kHz.
 *
 * The output can drive an optical transmitter (TOTX173/177) or coax driver.
 */

typedef struct parlio_adat_tx *parlio_adat_tx_handle_t;

typedef struct {
    uint32_t sample_rate;       /* target sample rate (typically 48000) */
    uint16_t mclk_multiple;     /* MCLK/Fs ratio for APLL, must be >= 256, default 512 */

    gpio_num_t mclk_gpio;       /* MCLK output (I2S/APLL) + PARLIO ext clk input */
    gpio_num_t adat_gpio;       /* ADAT data output (PARLIO TXD[0]) */

    size_t dma_buffer_count;    /* number of DMA buffers (default: 4) */
    size_t frames_per_buffer;   /* audio frames per DMA buffer (default: 64) */
} parlio_adat_tx_config_t;

esp_err_t parlio_adat_tx_new(const parlio_adat_tx_config_t *config,
                              parlio_adat_tx_handle_t *ret_handle);

esp_err_t parlio_adat_tx_enable(parlio_adat_tx_handle_t handle);
esp_err_t parlio_adat_tx_disable(parlio_adat_tx_handle_t handle);

/**
 * Write 8-channel audio samples. Each frame = 8 x int32_t [ch0..ch7],
 * left-justified (only upper 24 bits used).
 */
esp_err_t parlio_adat_tx_write(parlio_adat_tx_handle_t handle,
                                const int32_t *samples,
                                size_t num_frames,
                                size_t *frames_written,
                                uint32_t timeout_ms);

esp_err_t parlio_adat_tx_delete(parlio_adat_tx_handle_t handle);

#ifdef __cplusplus
}
#endif
