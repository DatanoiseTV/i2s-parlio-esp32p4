#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SPDIF transmitter using PARLIO on ESP32-P4.
 *
 * Outputs an IEC60958 / S/PDIF bitstream on a single GPIO pin using
 * biphase mark coding (BMC). Clocked by the APLL via I2S MCLK loopback.
 *
 * PARLIO runs at 128 * Fs (the biphase symbol rate):
 *   - Each audio bit becomes 2 symbols in BMC
 *   - 64 bits per subframe * 2 subframes per frame * 2 symbols per bit = 256 symbols/frame
 *   - PARLIO clock = 256 * Fs = 128 * Fs * 2 (but since each bit = 2 symbols,
 *     and there are 128 bits per frame, clock = 128 * Fs)
 *
 * Actually: each frame = 2 subframes * 32 bits = 64 bits. In BMC, each bit
 * is 2 UI (unit intervals). So one frame = 128 UI. Clock rate = 128 * Fs.
 *
 * At 48 kHz: PARLIO clock = 6.144 MHz
 * At 96 kHz: PARLIO clock = 12.288 MHz
 * At 192 kHz: PARLIO clock = 24.576 MHz
 *
 * Channel status block: 192 frames, transmitted cyclically.
 */

typedef struct parlio_spdif_tx *parlio_spdif_tx_handle_t;

typedef struct {
    uint32_t sample_rate;       /* target sample rate in Hz */
    uint8_t  bits_per_sample;   /* 16, 20, or 24 */
    uint16_t mclk_multiple;     /* MCLK/Fs ratio for APLL, must be >= 128, default 256 */

    gpio_num_t mclk_gpio;       /* MCLK output (I2S/APLL) + PARLIO ext clk input */
    gpio_num_t spdif_gpio;      /* S/PDIF data output (PARLIO TXD[0]) */

    /* Optional: set to true for consumer format, false for professional */
    bool consumer_format;

    size_t dma_buffer_count;    /* number of DMA buffers (default: 4) */
    size_t frames_per_buffer;   /* audio frames per DMA buffer (default: 192 = one channel status block) */
} parlio_spdif_tx_config_t;

esp_err_t parlio_spdif_tx_new(const parlio_spdif_tx_config_t *config,
                               parlio_spdif_tx_handle_t *ret_handle);

esp_err_t parlio_spdif_tx_enable(parlio_spdif_tx_handle_t handle);
esp_err_t parlio_spdif_tx_disable(parlio_spdif_tx_handle_t handle);

/**
 * Write stereo audio samples. Each frame = [left, right] as int32_t,
 * left-justified regardless of bits_per_sample.
 */
esp_err_t parlio_spdif_tx_write(parlio_spdif_tx_handle_t handle,
                                 const int32_t *samples,
                                 size_t num_frames,
                                 size_t *frames_written,
                                 uint32_t timeout_ms);

esp_err_t parlio_spdif_tx_delete(parlio_spdif_tx_handle_t handle);

#ifdef __cplusplus
}
#endif
