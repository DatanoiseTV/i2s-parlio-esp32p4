#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"

#include "parlio_i2s.h"

static const char *TAG = "audio_demo";

/*
 * Standalone parlio_i2s driver test: I2S stereo at 48kHz.
 * Wire: GPIO 20 -> GPIO 21 (MCLK loopback)
 */

#define PIN_MCLK       GPIO_NUM_21  /* PARLIO ext clk in (wire destination) */
#define PIN_BCLK       GPIO_NUM_22  /* clk_out = BCLK */
#define PIN_LRCK       GPIO_NUM_23  /* TXD[0] */
#define PIN_DATA       GPIO_NUM_24  /* TXD[1] */

#define SAMPLE_RATE    48000
#define TEST_SECONDS   30
#define FRAMES_PER_BUF 4096

static uint32_t ramp_inc(uint32_t freq_hz)
{
    return (uint32_t)(((uint64_t)freq_hz << 32) / SAMPLE_RATE);
}

typedef struct {
    parlio_i2s_tx_handle_t tx;
    volatile uint32_t total_frames;
    volatile bool running;
} ctx_t;

static void audio_task(void *arg)
{
    ctx_t *ctx = (ctx_t *)arg;

    int32_t *buf = malloc(FRAMES_PER_BUF * 2 * sizeof(int32_t));
    if (!buf) { ctx->running = false; vTaskDelete(NULL); return; }

    uint32_t ph[2] = {0};
    const uint32_t inc0 = ramp_inc(440);
    const uint32_t inc1 = ramp_inc(880);

    while (ctx->running) {
        for (size_t f = 0; f < FRAMES_PER_BUF; f++) {
            ph[0] += inc0;
            ph[1] += inc1;
            buf[f * 2 + 0] = (int32_t)ph[0];
            buf[f * 2 + 1] = (int32_t)ph[1];
        }

        size_t written = 0;
        parlio_i2s_tx_write(ctx->tx, buf, FRAMES_PER_BUF, &written, 1000);
        ctx->total_frames += written;
    }

    free(buf);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_ldo_channel_handle_t ldo4 = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 4, .voltage_mv = 3300,
        .flags = { .adjustable = false },
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo4));

    ESP_LOGI(TAG, "=== Standalone parlio_i2s stereo test ===");
    ESP_LOGI(TAG, "Wire: GPIO 20 -> GPIO 21");

    parlio_i2s_tx_config_t cfg = {
        .sample_rate    = SAMPLE_RATE,
        .bits_per_sample = 32,
        .slot_width     = 32,
        .num_data_lines = 1,
        .mclk_multiple  = 256,
        .mode           = PARLIO_I2S_MODE_STANDARD,
        .mclk_gpio      = PIN_MCLK,
        .bclk_gpio      = PIN_BCLK,
        .lrck_gpio      = PIN_LRCK,
        .data_gpios     = { PIN_DATA },
        .dma_buffer_count  = 4,
        .frames_per_buffer = FRAMES_PER_BUF,
    };

    parlio_i2s_tx_handle_t tx = NULL;
    ESP_ERROR_CHECK(parlio_i2s_tx_new(&cfg, &tx));
    ESP_ERROR_CHECK(parlio_i2s_tx_enable(tx));

    ESP_LOGI(TAG, "Fs=%d, 32-bit stereo, %d frames/buf", SAMPLE_RATE, FRAMES_PER_BUF);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Running %d-second test (encoder on CPU1) ===", TEST_SECONDS);

    static ctx_t ctx;
    ctx.tx = tx;
    ctx.total_frames = 0;
    ctx.running = true;

    xTaskCreatePinnedToCore(audio_task, "audio", 8192, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);

    for (int i = 0; i < TEST_SECONDS / 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        int elapsed = (i + 1) * 5;
        float rate = (float)ctx.total_frames / (float)elapsed;
        ESP_LOGI(TAG, "[%2ds] %"PRIu32" frames (%.0f Hz)", elapsed, ctx.total_frames, rate);
    }

    ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(100));

    uint32_t final = ctx.total_frames;
    uint32_t eff = final / TEST_SECONDS;
    float pct = 100.0f * (float)eff / (float)SAMPLE_RATE;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Done: %"PRIu32" frames in %d seconds ===", final, TEST_SECONDS);
    ESP_LOGI(TAG, "Effective rate: %"PRIu32" Hz (target: %d Hz)", eff, SAMPLE_RATE);
    if (pct >= 99.9f)
        ESP_LOGI(TAG, "PASS -- %.2f%%", pct);
    else if (pct >= 99.0f)
        ESP_LOGI(TAG, "PASS -- %.2f%% (FreeRTOS scheduling overhead)", pct);
    else
        ESP_LOGW(TAG, "UNDERRUN -- %.2f%%", pct);

    ESP_LOGI(TAG, "Continuing output...");
    ctx.running = true;
    xTaskCreatePinnedToCore(audio_task, "audio2", 8192, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);
    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
