#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "parlio_i2s.h"

static const char *TAG = "main";

/*
 * Example: 4-channel I2S TX (2 stereo pairs / 2 data lines) at 48 kHz, 32-bit,
 * driven by APLL through the PARLIO peripheral.
 *
 * GPIO assignments (adjust to your board):
 *   MCLK       = GPIO 10   (master clock output to DAC)
 *   APLL feed  = GPIO 11   (I2S MCLK -> PARLIO ext clk loopback)
 *   BCLK       = GPIO 12   (bit clock)
 *   LRCK       = GPIO 13   (word select / frame sync)
 *   DATA0      = GPIO 14   (stereo pair 0: L0/R0)
 *   DATA1      = GPIO 15   (stereo pair 1: L1/R1)
 */
#define PIN_MCLK       GPIO_NUM_10
#define PIN_APLL_FEED  GPIO_NUM_11
#define PIN_BCLK       GPIO_NUM_12
#define PIN_LRCK       GPIO_NUM_13
#define PIN_DATA0      GPIO_NUM_14
#define PIN_DATA1      GPIO_NUM_15

#define SAMPLE_RATE    48000
#define BITS           32
#define NUM_DATA_LINES 2
#define MCLK_MULT      256

/* Sine wave generator state */
#define TONE_HZ_L0     440.0f
#define TONE_HZ_R0     880.0f
#define TONE_HZ_L1     1000.0f
#define TONE_HZ_R1     1500.0f

static float phase[4] = {0};
static const float freqs[4] = { TONE_HZ_L0, TONE_HZ_R0, TONE_HZ_L1, TONE_HZ_R1 };

/* Generate one buffer of test tones. Samples are left-justified int32_t. */
static void generate_test_audio(int32_t *buf, size_t num_frames)
{
    const float two_pi = 2.0f * (float)M_PI;
    const float phase_inc_base = two_pi / (float)SAMPLE_RATE;
    const float amplitude = 0.5f; /* -6 dBFS */

    for (size_t f = 0; f < num_frames; f++) {
        for (int ch = 0; ch < 4; ch++) {
            float sample = amplitude * sinf(phase[ch]);
            /* Convert float [-1, 1] to left-justified int32_t */
            int32_t ival = (int32_t)(sample * (float)INT32_MAX);
            buf[f * 4 + ch] = ival;
            phase[ch] += phase_inc_base * freqs[ch];
            if (phase[ch] >= two_pi) phase[ch] -= two_pi;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "PARLIO I2S TX -- APLL-clocked multi-channel transmitter");
    ESP_LOGI(TAG, "Fs=%d, %d-bit, %d data lines, MCLK=%dx",
             SAMPLE_RATE, BITS, NUM_DATA_LINES, MCLK_MULT);

    parlio_i2s_tx_config_t cfg = {
        .sample_rate    = SAMPLE_RATE,
        .bits_per_sample = BITS,
        .slot_width     = BITS,
        .num_data_lines = NUM_DATA_LINES,
        .mclk_multiple  = MCLK_MULT,
        .mclk_gpio      = PIN_MCLK,
        .apll_feed_gpio = PIN_APLL_FEED,
        .bclk_gpio      = PIN_BCLK,
        .lrck_gpio      = PIN_LRCK,
        .data_gpios     = { PIN_DATA0, PIN_DATA1 },
        .dma_buffer_count  = 4,
        .frames_per_buffer = 128,
    };

    parlio_i2s_tx_handle_t tx = NULL;
    ESP_ERROR_CHECK(parlio_i2s_tx_new(&cfg, &tx));
    ESP_ERROR_CHECK(parlio_i2s_tx_enable(tx));

    ESP_LOGI(TAG, "transmitter running, generating test tones");

    /* Allocate a working buffer for audio generation */
    const size_t frames_per_write = 64;
    const size_t channels = NUM_DATA_LINES * 2;
    int32_t *audio_buf = malloc(frames_per_write * channels * sizeof(int32_t));
    if (!audio_buf) {
        ESP_LOGE(TAG, "failed to allocate audio buffer");
        return;
    }

    while (1) {
        generate_test_audio(audio_buf, frames_per_write);

        size_t written = 0;
        esp_err_t ret = parlio_i2s_tx_write(tx, audio_buf, frames_per_write,
                                             &written, 1000);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "write timeout, only wrote %u frames", (unsigned)written);
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "write error: %s", esp_err_to_name(ret));
            break;
        }
    }

    /* Cleanup (unreachable in this demo) */
    free(audio_buf);
    parlio_i2s_tx_disable(tx);
    parlio_i2s_tx_delete(tx);
}
