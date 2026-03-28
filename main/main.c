#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

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

/* Set up PCNT to count BCLK rising edges on the clk_out pin.
 * Uses accum_count to handle 16-bit overflow at high BCLK rates. */
static pcnt_unit_handle_t setup_pcnt(gpio_num_t bclk_gpio)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -1,
        .high_limit = 32767,
        .flags.accum_count = 1, /* accumulate across overflows */
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num  = bclk_gpio,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_cfg, &chan));

    /* Count rising edges only */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD));

    /* Enable input on the output pin so PCNT can read it */
    gpio_input_enable(bclk_gpio);

    /* Add watch point at high_limit for accumulation to work */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, 32767));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));

    return unit;
}

static void run_test(const char *label, parlio_i2s_mode_t mode,
                     uint8_t num_data_lines, uint16_t mclk_mult)
{
    uint8_t num_slots = (uint8_t)mode;
    int num_channels = num_data_lines * num_slots;
    uint32_t expected_bclk = (uint32_t)32 * num_slots * SAMPLE_RATE;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== %s: %d lines x %d slots = %d ch (BCLK=%.3f MHz) ===",
             label, num_data_lines, num_slots, num_channels,
             expected_bclk / 1e6f);

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
        ESP_LOGE(TAG, "  SKIP -- init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_ERROR_CHECK(parlio_i2s_tx_enable(tx));

    /* Set up PCNT to count actual BCLK edges on the wire */
    pcnt_unit_handle_t pcnt = setup_pcnt(PIN_BCLK);

    /* Start audio task on CPU1 */
    static ctx_t ctx;
    ctx.tx = tx;
    ctx.num_channels = num_channels;
    ctx.total_frames = 0;
    ctx.running = true;

    xTaskCreatePinnedToCore(audio_task, "audio", 8192, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);

    /* Measure for TEST_SECONDS using PCNT */
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt));
    int64_t t_start = esp_timer_get_time();

    for (int i = 0; i < TEST_SECONDS / 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        int elapsed = (i + 1) * 5;

        int pcnt_val = 0;
        pcnt_unit_get_count(pcnt, &pcnt_val);
        int64_t t_now = esp_timer_get_time();
        double dt = (t_now - t_start) / 1e6;
        double bclk_hz = pcnt_val / dt;
        double fs_hz = bclk_hz / (32 * num_slots);

        ESP_LOGI(TAG, "  [%2ds] SW=%"PRIu32" frames (%.0f Hz) | PCNT=%d edges (BCLK=%.0f Hz, Fs=%.1f Hz)",
                 elapsed, ctx.total_frames,
                 (float)ctx.total_frames / (float)elapsed,
                 pcnt_val, bclk_hz, fs_hz);
    }

    /* Final measurement */
    ESP_ERROR_CHECK(pcnt_unit_stop(pcnt));
    int final_count = 0;
    pcnt_unit_get_count(pcnt, &final_count);
    int64_t t_end = esp_timer_get_time();
    double total_sec = (t_end - t_start) / 1e6;
    double final_bclk = final_count / total_sec;
    double final_fs = final_bclk / (32 * num_slots);

    ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t sw_frames = ctx.total_frames;
    double sw_fs = sw_frames / total_sec;

    ESP_LOGI(TAG, "  ---");
    ESP_LOGI(TAG, "  SW encoder: %"PRIu32" frames in %.2fs = %.1f Hz (%.2f%%)",
             sw_frames, total_sec, sw_fs, 100.0 * sw_fs / SAMPLE_RATE);
    ESP_LOGI(TAG, "  HW PCNT:    %d BCLK edges in %.2fs = %.1f Hz -> Fs=%.1f Hz (%.4f%%)",
             final_count, total_sec, final_bclk, final_fs,
             100.0 * final_fs / SAMPLE_RATE);

    if (final_fs >= 47999.0 && final_fs <= 48001.0)
        ESP_LOGI(TAG, "  PASS -- hardware-verified 48000 Hz");
    else if (final_fs >= 47900.0 && final_fs <= 48100.0)
        ESP_LOGW(TAG, "  OK -- within 0.2%% of target");
    else
        ESP_LOGE(TAG, "  FAIL -- %.1f Hz off target", final_fs - 48000.0);

    /* Cleanup */
    pcnt_unit_disable(pcnt);
    pcnt_del_unit(pcnt);
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

    ESP_LOGI(TAG, "=== PARLIO I2S throughput sweep (PCNT-verified) ===");
    ESP_LOGI(TAG, "Wire: GPIO 20 -> GPIO 21, Fs=%d, 32-bit, %d frames/buf",
             SAMPLE_RATE, FRAMES_PER_BUF);

    run_test("I2S Stereo",  PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo x2",   PARLIO_I2S_MODE_STANDARD, 2, 256);
    run_test("TDM4 x1",     PARLIO_I2S_MODE_TDM4,     1, 256);
    run_test("TDM8 x1",     PARLIO_I2S_MODE_TDM8,     1, 512);
    run_test("TDM4 x2",     PARLIO_I2S_MODE_TDM4,     2, 256);
    run_test("TDM8 x2",     PARLIO_I2S_MODE_TDM8,     2, 512);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== All tests complete ===");
    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
