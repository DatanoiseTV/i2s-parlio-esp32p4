#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"

#include "parlio_i2s.h"

static const char *TAG = "audio_test";

#define PIN_MCLK       GPIO_NUM_21
#define PIN_BCLK       GPIO_NUM_22
#define PIN_LRCK       GPIO_NUM_23
#define PIN_DATA0      GPIO_NUM_24
#define PIN_DATA1      GPIO_NUM_25

#define SAMPLE_RATE    48000
#define TEST_SECONDS   10
#define FRAMES_PER_BUF 128

static uint32_t ramp_inc(uint32_t freq_hz)
{
    return (uint32_t)(((uint64_t)freq_hz << 32) / SAMPLE_RATE);
}

typedef struct {
    parlio_i2s_tx_handle_t tx;
    int num_channels;
    volatile uint32_t total_frames;
    volatile bool running;
} ctx_t;

static void audio_task(void *arg)
{
    ctx_t *ctx = (ctx_t *)arg;
    const int nch = ctx->num_channels;
    const size_t buf_samples = FRAMES_PER_BUF * nch;

    int32_t *buf = malloc(buf_samples * sizeof(int32_t));
    if (!buf) { ctx->running = false; vTaskDelete(NULL); return; }

    uint32_t ph[16] = {0};
    uint32_t inc[16];
    for (int ch = 0; ch < nch && ch < 16; ch++)
        inc[ch] = ramp_inc(200 + ch * 100);

    while (ctx->running) {
        for (size_t f = 0; f < FRAMES_PER_BUF; f++) {
            int base = f * nch;
            for (int ch = 0; ch < nch; ch++) {
                ph[ch] += inc[ch];
                buf[base + ch] = (int32_t)ph[ch];
            }
        }
        size_t written = 0;
        parlio_i2s_tx_write(ctx->tx, buf, FRAMES_PER_BUF, &written, 1000);
        ctx->total_frames += written;
    }

    free(buf);
    vTaskDelete(NULL);
}

static void run_test(const char *label, parlio_i2s_mode_t mode,
                     uint8_t num_data_lines, uint16_t mclk_mult)
{
    uint8_t num_slots = (uint8_t)mode;
    int num_channels = num_data_lines * num_slots;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== %s: %d lines x %d slots = %d channels ===",
             label, num_data_lines, num_slots, num_channels);

    parlio_i2s_tx_config_t cfg = {
        .sample_rate    = SAMPLE_RATE,
        .bits_per_sample = 32,
        .slot_width     = 32,
        .num_data_lines = num_data_lines,
        .mclk_multiple  = mclk_mult,
        .mode           = mode,
        .mclk_gpio      = PIN_MCLK,
        .bclk_gpio      = PIN_BCLK,
        .lrck_gpio      = PIN_LRCK,
        .data_gpios     = { PIN_DATA0, PIN_DATA1 },
        .dma_buffer_count  = 4,
        .frames_per_buffer = FRAMES_PER_BUF,
    };

    parlio_i2s_tx_handle_t tx = NULL;
    esp_err_t err = parlio_i2s_tx_new(&cfg, &tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  SKIP -- driver init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_ERROR_CHECK(parlio_i2s_tx_enable(tx));

    static ctx_t ctx;
    ctx.tx = tx;
    ctx.num_channels = num_channels;
    ctx.total_frames = 0;
    ctx.running = true;

    TaskHandle_t task = NULL;
    xTaskCreatePinnedToCore(audio_task, "audio", 8192, &ctx,
                            configMAX_PRIORITIES - 1, &task, 1);

    for (int i = 0; i < TEST_SECONDS / 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        int elapsed = (i + 1) * 5;
        ESP_LOGI(TAG, "  [%2ds] %"PRIu32" frames (%.0f Hz)",
                 elapsed, ctx.total_frames,
                 (float)ctx.total_frames / (float)elapsed);
    }

    ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(200));

    uint32_t final = ctx.total_frames;
    uint32_t eff = final / TEST_SECONDS;
    float pct = 100.0f * (float)eff / (float)SAMPLE_RATE;

    ESP_LOGI(TAG, "  Result: %"PRIu32" Hz (%.2f%%) -- %s",
             eff, pct, pct >= 99.9f ? "PASS" : (pct >= 99.0f ? "OK" : "UNDERRUN"));

    /* Clean up */
    if (task) vTaskDelete(task);
    vTaskDelay(pdMS_TO_TICKS(100));
    parlio_i2s_tx_disable(tx);
    parlio_i2s_tx_delete(tx);
    vTaskDelay(pdMS_TO_TICKS(200));
}

void app_main(void)
{
    esp_ldo_channel_handle_t ldo4 = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 4, .voltage_mv = 3300,
        .flags = { .adjustable = false },
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo4));

    ESP_LOGI(TAG, "=== PARLIO I2S throughput sweep ===");
    ESP_LOGI(TAG, "Wire: GPIO 20 -> GPIO 21, Fs=%d, 32-bit, %d frames/buf",
             SAMPLE_RATE, FRAMES_PER_BUF);

    /* Test 1: Stereo (2 channels, 1 line, BCLK=64*Fs) */
    run_test("I2S Stereo", PARLIO_I2S_MODE_STANDARD, 1, 256);

    /* Test 2: Stereo x2 lines (4 channels, BCLK=64*Fs) */
    run_test("I2S Stereo x2", PARLIO_I2S_MODE_STANDARD, 2, 256);

    /* Test 3: TDM4 1 line (4 channels, BCLK=128*Fs) */
    run_test("TDM4 x1", PARLIO_I2S_MODE_TDM4, 1, 256);

    /* Test 4: TDM8 1 line (8 channels, BCLK=256*Fs) */
    run_test("TDM8 x1", PARLIO_I2S_MODE_TDM8, 1, 512);

    /* Test 5: TDM4 2 lines (8 channels, BCLK=128*Fs) */
    run_test("TDM4 x2", PARLIO_I2S_MODE_TDM4, 2, 256);

    /* Test 6: TDM8 2 lines (16 channels, BCLK=256*Fs) */
    run_test("TDM8 x2", PARLIO_I2S_MODE_TDM8, 2, 512);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== All tests complete ===");
    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
