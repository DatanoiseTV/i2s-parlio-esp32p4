#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "parlio_audio_tx.h"

static const char *TAG = "audio_demo";

/*
 * TDM8 test via unified driver on the GPIO 20<->21 loopback wire.
 *
 * 1 data line x 8 TDM slots = 8 channels at 48 kHz, 32-bit.
 * I2S-only mode: PARLIO runs at BCLK rate (256*Fs with slot_width=32, 8 slots),
 * BCLK on dedicated clk_out pin. Efficient path, no ADAT/SPDIF overhead.
 *
 * GPIO 20 = MCLK out (wire source)
 * GPIO 21 = PARLIO ext clk in (wire destination)
 * GPIO 22 = BCLK (clk_out)
 * GPIO 23 = LRCK / frame sync (TXD[0])
 * GPIO 24 = DATA (TXD[1])
 */

#define PIN_MCLK       GPIO_NUM_20
#define PIN_PARLIO_CLK GPIO_NUM_21
#define PIN_CLK_OUT    GPIO_NUM_22
#define PIN_LRCK       GPIO_NUM_23
#define PIN_DATA       GPIO_NUM_24

#define SAMPLE_RATE    48000
#define TEST_SECONDS   30
#define FRAMES_PER_WRITE 256

static inline __attribute__((always_inline)) int32_t ramp_sample(uint32_t *phase, uint32_t freq_hz)
{
    uint32_t inc = (uint32_t)(((uint64_t)freq_hz << 32) / SAMPLE_RATE);
    *phase += inc;
    return (int32_t)*phase;
}

typedef struct {
    parlio_audio_tx_handle_t tx;
    size_t frame_size;
    volatile uint32_t total_frames;
    volatile bool running;
} audio_ctx_t;

static void audio_task(void *arg)
{
    audio_ctx_t *ctx = (audio_ctx_t *)arg;
    const size_t fpw = FRAMES_PER_WRITE;
    const size_t fs = ctx->frame_size;

    int32_t *buf = malloc(fpw * fs * sizeof(int32_t));
    if (!buf) {
        ESP_LOGE(TAG, "alloc failed");
        ctx->running = false;
        vTaskDelete(NULL);
        return;
    }

    /* 2 channels (stereo) */
    uint32_t phase[2] = {0};
    const int nch = (int)fs; /* frame_size = num channels */

    while (ctx->running) {
        for (size_t f = 0; f < fpw; f++) {
            size_t base = f * fs;
            for (int ch = 0; ch < nch; ch++)
                buf[base + ch] = ramp_sample(&phase[ch % 2], 440 + ch * 440);
        }

        size_t written = 0;
        parlio_audio_tx_write(ctx->tx, buf, fpw, &written, 1000);
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

    ESP_LOGI(TAG, "=== I2S stereo test: 1 line x 2 slots = 2 channels ===");
    ESP_LOGI(TAG, "Wire: GPIO %d -> GPIO %d", PIN_MCLK, PIN_PARLIO_CLK);

    parlio_audio_i2s_config_t i2s_cfg = {
        .mode = PARLIO_AUDIO_I2S_STANDARD,
        .bits_per_sample = 32,
        .slot_width = 32,
        .num_data_lines = 1,
        .bclk_gpio = PIN_CLK_OUT,
        .lrck_gpio = PIN_LRCK,
        .data_gpios = { PIN_DATA },
    };

    parlio_audio_tx_config_t cfg = {
        .sample_rate = SAMPLE_RATE,
        .mclk_multiple = 256,       /* MCLK = 12.288 MHz, BCLK = 3.072 MHz (64*Fs) */
        .mclk_gpio = PIN_PARLIO_CLK,
        .clk_out_gpio = PIN_CLK_OUT,
        .i2s = &i2s_cfg,
        .dma_buffer_count = 4,
        .frames_per_buffer = FRAMES_PER_WRITE,
    };

    parlio_audio_tx_handle_t tx = NULL;
    ESP_ERROR_CHECK(parlio_audio_tx_new(&cfg, &tx));
    ESP_ERROR_CHECK(parlio_audio_tx_enable(tx));

    size_t fs = parlio_audio_tx_get_frame_size(tx);
    uint32_t clk = parlio_audio_tx_get_parlio_clock(tx);

    ESP_LOGI(TAG, "PARLIO clock: %"PRIu32" Hz, frame size: %u samples, BCLK on clk_out: %s",
             clk, (unsigned)fs, (clk == (uint32_t)(32 * 8 * SAMPLE_RATE)) ? "yes" : "no");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Running %d-second test (encoder on CPU1) ===", TEST_SECONDS);
    ESP_LOGI(TAG, "  %u channels, sawtooth", (unsigned)fs);

    static audio_ctx_t ctx;
    ctx.tx = tx;
    ctx.frame_size = fs;
    ctx.total_frames = 0;
    ctx.running = true;

    xTaskCreatePinnedToCore(audio_task, "audio_enc", 8192, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);

    for (int i = 0; i < TEST_SECONDS / 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        int elapsed = (int)((i + 1) * 5);
        float rate = (float)ctx.total_frames / (float)elapsed;
        ESP_LOGI(TAG, "[%2ds] %"PRIu32" frames (%.0f Hz)", elapsed, ctx.total_frames, rate);
    }

    ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(100));

    uint32_t final = ctx.total_frames;
    uint32_t eff = final / TEST_SECONDS;
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Done: %"PRIu32" frames in %d seconds ===", final, TEST_SECONDS);
    ESP_LOGI(TAG, "Effective rate: %"PRIu32" Hz (target: %d Hz)", eff, SAMPLE_RATE);

    float pct = 100.0f * (float)eff / (float)SAMPLE_RATE;
    if (pct >= 99.0f)
        ESP_LOGI(TAG, "PASS -- real-time TDM8 output at %.1f%%", pct);
    else
        ESP_LOGW(TAG, "UNDERRUN -- %.1f%% of target rate", pct);

    ESP_LOGI(TAG, "Continuing output for scope inspection...");
    ctx.running = true;
    xTaskCreatePinnedToCore(audio_task, "audio_cont", 8192, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);
    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
