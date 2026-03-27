#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2s_common.h"
#include "esp_rom_gpio.h"
#include "soc/parlio_periph.h"

#include "parlio_audio_tx.h"

static const char *TAG = "audio_demo";

/*
 * Unified audio output demo: I2S stereo + ADAT 8-channel via PARLIO,
 * plus I2S HW passthrough stereo on the clock-generator peripheral.
 *
 * Physical wire: GPIO 20 -> GPIO 21 (MCLK loopback)
 *
 * PARLIO outputs (at 256*Fs = 12.288 MHz):
 *   TXD[0] = I2S BCLK (synthesized, 64*Fs)
 *   TXD[1] = I2S LRCK
 *   TXD[2] = I2S DATA (1 stereo pair)
 *   TXD[3] = ADAT bitstream (8 channels, NRZI encoded)
 *   clk_out = PARLIO clock (256*Fs, usable as MCLK)
 *
 * I2S HW (on the clock-generator I2S peripheral):
 *   BCLK = GPIO 25, WS = GPIO 26, DOUT = GPIO 27
 *   Standard stereo, 32-bit, same APLL clock
 *
 * Total: 2 (PARLIO I2S) + 8 (PARLIO ADAT) + 2 (I2S HW) = 12 channels
 */

#define PIN_MCLK       GPIO_NUM_20
#define PIN_PARLIO_CLK GPIO_NUM_21  /* wired to PIN_MCLK */
#define PIN_CLK_OUT    GPIO_NUM_22  /* PARLIO clock output (256*Fs) */
#define PIN_I2S_BCLK   GPIO_NUM_23  /* PARLIO I2S BCLK (synthesized on TXD) */
#define PIN_I2S_LRCK   GPIO_NUM_24  /* PARLIO I2S LRCK */
#define PIN_I2S_DATA   GPIO_NUM_25  /* PARLIO I2S data */
#define PIN_ADAT       GPIO_NUM_26  /* PARLIO ADAT output */
#define PIN_HW_BCLK    GPIO_NUM_27  /* I2S HW peripheral BCLK */
#define PIN_HW_WS      GPIO_NUM_28  /* I2S HW peripheral WS */
#define PIN_HW_DOUT    GPIO_NUM_29  /* I2S HW peripheral DOUT */

#define SAMPLE_RATE    48000
#define TEST_SECONDS   30

/* Generate a sine tone in left-justified int32_t */
static int32_t sine_sample(float *phase, float freq, float amplitude)
{
    float s = amplitude * sinf(*phase);
    *phase += 2.0f * (float)M_PI * freq / (float)SAMPLE_RATE;
    if (*phase >= 2.0f * (float)M_PI) *phase -= 2.0f * (float)M_PI;
    return (int32_t)(s * (float)INT32_MAX);
}

void app_main(void)
{
    /* Power GPIO bank for GPIO 20+ */
    esp_ldo_channel_handle_t ldo4 = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 4, .voltage_mv = 3300,
        .flags = { .adjustable = false },
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo4));

    ESP_LOGI(TAG, "=== Unified audio output: I2S + ADAT + I2S HW ===");
    ESP_LOGI(TAG, "Wire: GPIO %d -> GPIO %d", PIN_MCLK, PIN_PARLIO_CLK);

    /* --- Configure all outputs --- */

    /* PARLIO I2S: 1 stereo pair (2 channels) */
    parlio_audio_i2s_config_t i2s_cfg = {
        .mode = PARLIO_AUDIO_I2S_STANDARD,
        .bits_per_sample = 32,
        .slot_width = 32,
        .num_data_lines = 1,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .data_gpios = { PIN_I2S_DATA },
    };

    /* PARLIO ADAT: 8 channels */
    parlio_audio_adat_config_t adat_cfg = {
        .adat_gpio = PIN_ADAT,
    };

    /* I2S HW passthrough: stereo on the clock-generator peripheral */
    parlio_audio_i2s_hw_config_t hw_cfg = {
        .bits_per_sample = 32,
        .slot_width = 32,
        .total_slots = 0,              /* 0 = stereo (2 slots) */
        .bclk_gpio = PIN_HW_BCLK,
        .ws_gpio   = PIN_HW_WS,
        .dout_gpio = PIN_HW_DOUT,
        .dout2_gpio = -1,
    };

    /* Unified config: all three active simultaneously */
    parlio_audio_tx_config_t cfg = {
        .sample_rate    = SAMPLE_RATE,
        .mclk_multiple  = 512,         /* 512*Fs = 24.576 MHz MCLK, PARLIO divides to 256*Fs */
        .mclk_gpio      = PIN_PARLIO_CLK,
        .clk_out_gpio   = PIN_CLK_OUT,
        .i2s   = &i2s_cfg,
        .adat  = &adat_cfg,
        .i2s_hw = &hw_cfg,
        .dma_buffer_count  = 4,
        .frames_per_buffer = 64,
    };

    parlio_audio_tx_handle_t tx = NULL;
    ESP_ERROR_CHECK(parlio_audio_tx_new(&cfg, &tx));

    /* Set up the separate I2S MCLK output on GPIO 20 (the wire source).
     * The unified driver puts I2S MCLK on mclk_gpio (= PIN_PARLIO_CLK = 21),
     * which is the PARLIO input side. We need MCLK also driven on GPIO 20
     * (the wire output side). Since both GPIOs are wired together, the I2S
     * MCLK output on GPIO 21 drives GPIO 20 through the wire. */

    ESP_ERROR_CHECK(parlio_audio_tx_enable(tx));

    size_t parlio_frame_size = parlio_audio_tx_get_frame_size(tx);
    uint32_t parlio_clock = parlio_audio_tx_get_parlio_clock(tx);
    i2s_chan_handle_t i2s_hw = parlio_audio_tx_get_i2s_hw_handle(tx);

    ESP_LOGI(TAG, "PARLIO clock: %"PRIu32" Hz", parlio_clock);
    ESP_LOGI(TAG, "PARLIO frame size: %u samples (I2S:2 + ADAT:8 = 10)", (unsigned)parlio_frame_size);
    ESP_LOGI(TAG, "I2S HW handle: %s", i2s_hw ? "active" : "NULL");

    /* --- Allocate audio buffers --- */
    const size_t frames_per_write = 32;

    /* PARLIO buffer: [2 I2S samples] [8 ADAT samples] = 10 per frame */
    int32_t *parlio_buf = malloc(frames_per_write * parlio_frame_size * sizeof(int32_t));

    /* I2S HW buffer: stereo = 2 samples per frame */
    int32_t *hw_buf = malloc(frames_per_write * 2 * sizeof(int32_t));

    if (!parlio_buf || !hw_buf) {
        ESP_LOGE(TAG, "alloc failed");
        return;
    }

    /* --- Tone generator state --- */
    /* I2S: 440 Hz L, 880 Hz R */
    float i2s_phase[2] = {0};
    /* ADAT: 8 channels at 200-900 Hz */
    float adat_phase[8] = {0};
    /* I2S HW: 1000 Hz L, 1500 Hz R */
    float hw_phase[2] = {0};

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Generating audio for %d seconds ===", TEST_SECONDS);
    ESP_LOGI(TAG, "  PARLIO I2S:  440 Hz (L), 880 Hz (R)");
    ESP_LOGI(TAG, "  PARLIO ADAT: 200-900 Hz across 8 channels");
    ESP_LOGI(TAG, "  I2S HW:      1000 Hz (L), 1500 Hz (R)");

    int64_t start = esp_timer_get_time();
    int64_t end = start + (int64_t)TEST_SECONDS * 1000000;
    uint32_t total_frames = 0;

    while (esp_timer_get_time() < end) {
        /* Generate PARLIO audio (I2S + ADAT interleaved) */
        for (size_t f = 0; f < frames_per_write; f++) {
            size_t base = f * parlio_frame_size;

            /* I2S: 2 samples (L, R) */
            parlio_buf[base + 0] = sine_sample(&i2s_phase[0], 440.0f, 0.5f);
            parlio_buf[base + 1] = sine_sample(&i2s_phase[1], 880.0f, 0.5f);

            /* ADAT: 8 samples (ch0..ch7) */
            for (int ch = 0; ch < 8; ch++) {
                float freq = 200.0f + (float)ch * 100.0f;
                parlio_buf[base + 2 + ch] = sine_sample(&adat_phase[ch], freq, 0.25f);
            }
        }

        /* Generate I2S HW audio (separate buffer, separate write) */
        for (size_t f = 0; f < frames_per_write; f++) {
            hw_buf[f * 2 + 0] = sine_sample(&hw_phase[0], 1000.0f, 0.5f);
            hw_buf[f * 2 + 1] = sine_sample(&hw_phase[1], 1500.0f, 0.5f);
        }

        /* Write PARLIO path */
        size_t written = 0;
        esp_err_t ret = parlio_audio_tx_write(tx, parlio_buf, frames_per_write,
                                               &written, 1000);
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "PARLIO write error: %s", esp_err_to_name(ret));
            break;
        }

        /* Write I2S HW path */
        if (i2s_hw) {
            size_t hw_bytes = 0;
            i2s_channel_write(i2s_hw, hw_buf,
                              frames_per_write * 2 * sizeof(int32_t),
                              &hw_bytes, 1000);
        }

        total_frames += written;

        /* Periodic status */
        int elapsed = (int)((esp_timer_get_time() - start) / 1000000);
        if (elapsed > 0 && elapsed % 10 == 0 && total_frames % (SAMPLE_RATE * 10) < frames_per_write) {
            ESP_LOGI(TAG, "[%2ds] %"PRIu32" frames written", elapsed, total_frames);
        }
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Done: %"PRIu32" frames in %d seconds ===", total_frames, TEST_SECONDS);
    ESP_LOGI(TAG, "Effective rate: %"PRIu32" Hz (target: %d Hz)",
             total_frames / TEST_SECONDS, SAMPLE_RATE);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Continuing output for scope inspection...");

    /* Keep outputting */
    while (1) {
        for (size_t f = 0; f < frames_per_write; f++) {
            size_t base = f * parlio_frame_size;
            parlio_buf[base + 0] = sine_sample(&i2s_phase[0], 440.0f, 0.5f);
            parlio_buf[base + 1] = sine_sample(&i2s_phase[1], 880.0f, 0.5f);
            for (int ch = 0; ch < 8; ch++)
                parlio_buf[base + 2 + ch] = sine_sample(&adat_phase[ch],
                                                          200.0f + ch * 100.0f, 0.25f);
        }
        parlio_audio_tx_write(tx, parlio_buf, frames_per_write, NULL, 1000);

        if (i2s_hw) {
            for (size_t f = 0; f < frames_per_write; f++) {
                hw_buf[f * 2 + 0] = sine_sample(&hw_phase[0], 1000.0f, 0.5f);
                hw_buf[f * 2 + 1] = sine_sample(&hw_phase[1], 1500.0f, 0.5f);
            }
            size_t bw = 0;
            i2s_channel_write(i2s_hw, hw_buf,
                              frames_per_write * 2 * sizeof(int32_t), &bw, 1000);
        }
    }
}
