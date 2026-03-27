#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2s_common.h"

#include "parlio_audio_tx.h"

static const char *TAG = "audio_demo";

/*
 * Unified audio output demo: I2S + ADAT via PARLIO, plus I2S HW passthrough.
 * Encoding runs on a dedicated high-priority task pinned to CPU1.
 *
 * Physical wire: GPIO 20 -> GPIO 21 (MCLK loopback)
 */

#define PIN_MCLK       GPIO_NUM_20
#define PIN_PARLIO_CLK GPIO_NUM_21
#define PIN_CLK_OUT    GPIO_NUM_22
#define PIN_I2S_BCLK   GPIO_NUM_23
#define PIN_I2S_LRCK   GPIO_NUM_24
#define PIN_I2S_DATA   GPIO_NUM_25
#define PIN_ADAT       GPIO_NUM_26
#define PIN_HW_BCLK    GPIO_NUM_27
#define PIN_HW_WS      GPIO_NUM_28
#define PIN_HW_DOUT    GPIO_NUM_29

#define SAMPLE_RATE    48000
#define TEST_SECONDS   30
#define FRAMES_PER_WRITE 64

static int32_t ramp_sample(uint32_t *phase, uint32_t freq_hz)
{
    uint32_t inc = (uint32_t)(((uint64_t)freq_hz << 32) / SAMPLE_RATE);
    *phase += inc;
    return (int32_t)*phase;
}

/* Shared state between main and encoder task */
typedef struct {
    parlio_audio_tx_handle_t tx;
    i2s_chan_handle_t i2s_hw;
    size_t parlio_frame_size;
    volatile uint32_t total_frames;
    volatile bool running;
} audio_ctx_t;

/* Audio encoding task -- pinned to CPU1 for dedicated throughput */
static void audio_task(void *arg)
{
    audio_ctx_t *ctx = (audio_ctx_t *)arg;
    const size_t fpw = FRAMES_PER_WRITE;
    const size_t pfs = ctx->parlio_frame_size;

    int32_t *parlio_buf = malloc(fpw * pfs * sizeof(int32_t));
    int32_t *hw_buf = malloc(fpw * 2 * sizeof(int32_t));
    if (!parlio_buf || !hw_buf) {
        ESP_LOGE(TAG, "audio_task alloc failed");
        ctx->running = false;
        vTaskDelete(NULL);
        return;
    }

    uint32_t i2s_phase[2] = {0};
    uint32_t adat_phase[8] = {0};
    uint32_t hw_phase[2] = {0};

    while (ctx->running) {
        /* Generate samples */
        for (size_t f = 0; f < fpw; f++) {
            size_t base = f * pfs;
            parlio_buf[base + 0] = ramp_sample(&i2s_phase[0], 440);
            parlio_buf[base + 1] = ramp_sample(&i2s_phase[1], 880);
            for (int ch = 0; ch < 8; ch++)
                parlio_buf[base + 2 + ch] = ramp_sample(&adat_phase[ch], 200 + ch * 100);
        }

        /* Write PARLIO (blocks when DMA buffers full -- this is the flow control) */
        size_t written = 0;
        parlio_audio_tx_write(ctx->tx, parlio_buf, fpw, &written, 1000);
        ctx->total_frames += written;

        /* I2S HW write temporarily disabled to isolate PARLIO encoder throughput.
         * The i2s_channel_write blocks for up to 5ms (large DMA buffers),
         * starving the PARLIO encoder. Needs its own task for production. */
#if 0
        if (ctx->i2s_hw) {
            for (size_t f = 0; f < fpw; f++) {
                hw_buf[f * 2 + 0] = ramp_sample(&hw_phase[0], 1000);
                hw_buf[f * 2 + 1] = ramp_sample(&hw_phase[1], 1500);
            }
            size_t bw = 0;
            i2s_channel_write(ctx->i2s_hw, hw_buf,
                              fpw * 2 * sizeof(int32_t), &bw, 1000);
        }
#endif
    }

    free(parlio_buf);
    free(hw_buf);
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

    ESP_LOGI(TAG, "=== Unified audio output: I2S + ADAT + I2S HW ===");
    ESP_LOGI(TAG, "Wire: GPIO %d -> GPIO %d", PIN_MCLK, PIN_PARLIO_CLK);

    parlio_audio_i2s_config_t i2s_cfg = {
        .mode = PARLIO_AUDIO_I2S_STANDARD,
        .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 1,
        .bclk_gpio = PIN_I2S_BCLK, .lrck_gpio = PIN_I2S_LRCK,
        .data_gpios = { PIN_I2S_DATA },
    };
    parlio_audio_adat_config_t adat_cfg = { .adat_gpio = PIN_ADAT };
    parlio_audio_i2s_hw_config_t hw_cfg = {
        .bits_per_sample = 32, .slot_width = 32, .total_slots = 0,
        .bclk_gpio = PIN_HW_BCLK, .ws_gpio = PIN_HW_WS,
        .dout_gpio = PIN_HW_DOUT, .dout2_gpio = -1,
    };

    parlio_audio_tx_config_t cfg = {
        .sample_rate = SAMPLE_RATE, .mclk_multiple = 512,
        .mclk_gpio = PIN_PARLIO_CLK, .clk_out_gpio = PIN_CLK_OUT,
        .i2s = &i2s_cfg, .adat = &adat_cfg, .i2s_hw = &hw_cfg,
        .dma_buffer_count = 4, .frames_per_buffer = FRAMES_PER_WRITE,
    };

    parlio_audio_tx_handle_t tx = NULL;
    ESP_ERROR_CHECK(parlio_audio_tx_new(&cfg, &tx));
    ESP_ERROR_CHECK(parlio_audio_tx_enable(tx));

    size_t pfs = parlio_audio_tx_get_frame_size(tx);
    i2s_chan_handle_t i2s_hw = parlio_audio_tx_get_i2s_hw_handle(tx);

    ESP_LOGI(TAG, "PARLIO clock: %"PRIu32" Hz, frame size: %u samples",
             parlio_audio_tx_get_parlio_clock(tx), (unsigned)pfs);
    ESP_LOGI(TAG, "I2S HW: %s", i2s_hw ? "active" : "disabled");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Starting %d-second test (encoder on CPU1) ===", TEST_SECONDS);
    ESP_LOGI(TAG, "  PARLIO I2S:  440/880 Hz | ADAT: 200-900 Hz | I2S HW: 1000/1500 Hz");

    /* Launch encoding task on CPU1 */
    static audio_ctx_t ctx;
    ctx.tx = tx;
    ctx.i2s_hw = i2s_hw;
    ctx.parlio_frame_size = pfs;
    ctx.total_frames = 0;
    ctx.running = true;

    xTaskCreatePinnedToCore(audio_task, "audio_enc", 8192, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);

    /* Monitor from CPU0 */
    int64_t start = esp_timer_get_time();
    for (int sec = 0; sec < TEST_SECONDS; sec++) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        int elapsed = (int)((esp_timer_get_time() - start) / 1000000);
        ESP_LOGI(TAG, "[%2ds] %"PRIu32" frames (%.0f Hz)",
                 elapsed, ctx.total_frames,
                 (float)ctx.total_frames / ((float)elapsed > 0 ? elapsed : 1));
    }

    ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(100)); /* let task exit cleanly */

    uint32_t final = ctx.total_frames;
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Done: %"PRIu32" frames in %d seconds ===", final, TEST_SECONDS);
    ESP_LOGI(TAG, "Effective rate: %"PRIu32" Hz (target: %d Hz)",
             final / TEST_SECONDS, SAMPLE_RATE);

    float pct = 100.0f * (float)(final / TEST_SECONDS) / (float)SAMPLE_RATE;
    if (pct >= 99.0f) {
        ESP_LOGI(TAG, "PASS -- sustained real-time output at %.1f%% of target", pct);
    } else {
        ESP_LOGW(TAG, "UNDERRUN -- achieved %.1f%% of target rate", pct);
    }

    ESP_LOGI(TAG, "Continuing output for scope inspection...");
    ctx.running = true;
    xTaskCreatePinnedToCore(audio_task, "audio_cont", 8192, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);
    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
