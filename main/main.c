#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "parlio_i2s.h"

static const char *TAG = "main";

/*
 * Example: TDM8 output with 2 data lines = 16 channels total, at 48 kHz / 32-bit.
 * Driven by APLL through the PARLIO peripheral.
 *
 * New architecture: BCLK is output on the dedicated PARLIO clock-out pin,
 * freeing up one data line. MCLK comes directly from the I2S MCLK output
 * (same GPIO also feeds PARLIO as external clock input).
 *
 * GPIO assignments (adjust to your board):
 *   MCLK       = GPIO 10   (I2S MCLK output to DAC + PARLIO ext clk input)
 *   BCLK       = GPIO 11   (PARLIO clk_out, dedicated)
 *   LRCK       = GPIO 12   (PARLIO TXD[0], frame sync)
 *   DATA0      = GPIO 13   (PARLIO TXD[1], slots 0-7)
 *   DATA1      = GPIO 14   (PARLIO TXD[2], slots 8-15)
 *
 * For TDM8 at 48 kHz with 32-bit slots:
 *   BCLK = 32 * 8 * 48000 = 12.288 MHz
 *   MCLK = 512 * 48000    = 24.576 MHz (MCLK/BCLK = 2)
 *
 * Note on slot_width: set this to match what the connected codec expects.
 * Some codecs expect more BCLK cycles per frame than strictly needed for
 * the audio data (e.g., 256fs BCLK for a stereo 24-bit codec = slot_width=128).
 * The unused bit positions are zero-padded automatically.
 */

#define PIN_MCLK       GPIO_NUM_10
#define PIN_BCLK       GPIO_NUM_11
#define PIN_LRCK       GPIO_NUM_12
#define PIN_DATA0      GPIO_NUM_13
#define PIN_DATA1      GPIO_NUM_14

#define SAMPLE_RATE    48000
#define BITS           32
#define SLOT_WIDTH     32       /* adjust if codec expects wider slots */
#define NUM_DATA_LINES 2
#define NUM_SLOTS      8
#define TOTAL_CH       (NUM_DATA_LINES * NUM_SLOTS)  /* 16 */
#define MCLK_MULT      512

static float phase[TOTAL_CH] = {0};
static const float base_freq = 440.0f;

static void generate_test_audio(int32_t *buf, size_t num_frames)
{
    const float two_pi = 2.0f * (float)M_PI;
    const float phase_inc_base = two_pi / (float)SAMPLE_RATE;
    const float amplitude = 0.25f;

    for (size_t f = 0; f < num_frames; f++) {
        for (int ch = 0; ch < TOTAL_CH; ch++) {
            float freq = base_freq + (float)ch * 100.0f;
            float sample = amplitude * sinf(phase[ch]);
            buf[f * TOTAL_CH + ch] = (int32_t)(sample * (float)INT32_MAX);
            phase[ch] += phase_inc_base * freq;
            if (phase[ch] >= two_pi) phase[ch] -= two_pi;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "PARLIO I2S TX -- TDM8 mode, %d channels", TOTAL_CH);
    ESP_LOGI(TAG, "Fs=%d, %d-bit, %d-bit slots, %d data lines, %d slots/line, MCLK=%dx",
             SAMPLE_RATE, BITS, SLOT_WIDTH, NUM_DATA_LINES, NUM_SLOTS, MCLK_MULT);

    parlio_i2s_tx_config_t cfg = {
        .sample_rate    = SAMPLE_RATE,
        .bits_per_sample = BITS,
        .slot_width     = SLOT_WIDTH,
        .num_data_lines = NUM_DATA_LINES,
        .mclk_multiple  = MCLK_MULT,
        .mode           = PARLIO_I2S_MODE_TDM8,
        .mclk_gpio      = PIN_MCLK,
        .bclk_gpio      = PIN_BCLK,
        .lrck_gpio      = PIN_LRCK,
        .data_gpios     = { PIN_DATA0, PIN_DATA1 },
        .dma_buffer_count  = 4,
        .frames_per_buffer = 64,
    };

    parlio_i2s_tx_handle_t tx = NULL;
    ESP_ERROR_CHECK(parlio_i2s_tx_new(&cfg, &tx));
    ESP_ERROR_CHECK(parlio_i2s_tx_enable(tx));

    ESP_LOGI(TAG, "transmitter running, generating %d test tones", TOTAL_CH);

    const size_t frames_per_write = 32;
    int32_t *audio_buf = malloc(frames_per_write * TOTAL_CH * sizeof(int32_t));
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
            ESP_LOGW(TAG, "write timeout, wrote %u/%u frames",
                     (unsigned)written, (unsigned)frames_per_write);
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "write error: %s", esp_err_to_name(ret));
            break;
        }
    }

    free(audio_buf);
    parlio_i2s_tx_disable(tx);
    parlio_i2s_tx_delete(tx);
}
