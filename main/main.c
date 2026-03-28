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

#define TEST_SECONDS   5
#define FRAMES_PER_BUF 128

typedef struct {
    parlio_i2s_tx_handle_t tx;
    int num_channels;
    uint32_t sample_rate;
    volatile uint32_t total_frames;
    volatile bool running;
} ctx_t;

static void audio_task(void *arg)
{
    ctx_t *ctx = (ctx_t *)arg;
    const int nch = ctx->num_channels;

    int32_t *buf = malloc(FRAMES_PER_BUF * nch * sizeof(int32_t));
    if (!buf) { ctx->running = false; vTaskDelete(NULL); return; }

    uint32_t ph[16] = {0};
    uint32_t inc[16];
    for (int ch = 0; ch < nch && ch < 16; ch++)
        inc[ch] = (uint32_t)(((uint64_t)(200 + ch * 100) << 32) / ctx->sample_rate);

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

static pcnt_channel_handle_t s_pcnt_chan; /* stored for cleanup */

static pcnt_unit_handle_t setup_pcnt(gpio_num_t bclk_gpio)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -1,
        .high_limit = 32767,
        .flags.accum_count = 1,
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num  = bclk_gpio,
        .level_gpio_num = -1,
    };
    s_pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_cfg, &s_pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(s_pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD));

    gpio_input_enable(bclk_gpio);
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, 32767));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    return unit;
}

static void run_test(const char *label, uint32_t sample_rate,
                     parlio_i2s_mode_t mode, uint8_t num_data_lines,
                     uint16_t mclk_mult)
{
    uint8_t num_slots = (uint8_t)mode;
    int num_channels = num_data_lines * num_slots;
    uint32_t bclk = (uint32_t)32 * num_slots * sample_rate;

    ESP_LOGI(TAG, "  %-22s %5"PRIu32" Hz  %2d ch  BCLK=%6.3f MHz",
             label, sample_rate, num_channels, bclk / 1e6f);

    parlio_i2s_tx_config_t cfg = {
        .sample_rate    = sample_rate,
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
        ESP_LOGE(TAG, "    SKIP -- %s", esp_err_to_name(err));
        return;
    }
    ESP_ERROR_CHECK(parlio_i2s_tx_enable(tx));

    pcnt_unit_handle_t pcnt = setup_pcnt(PIN_BCLK);

    static ctx_t ctx;
    ctx.tx = tx;
    ctx.num_channels = num_channels;
    ctx.sample_rate = sample_rate;
    ctx.total_frames = 0;
    ctx.running = true;

    xTaskCreatePinnedToCore(audio_task, "audio", 8192, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);

    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt));
    int64_t t_start = esp_timer_get_time();

    vTaskDelay(pdMS_TO_TICKS(TEST_SECONDS * 1000));

    ESP_ERROR_CHECK(pcnt_unit_stop(pcnt));
    int final_count = 0;
    pcnt_unit_get_count(pcnt, &final_count);
    int64_t t_end = esp_timer_get_time();
    double dt = (t_end - t_start) / 1e6;

    ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(500));

    double hw_bclk = final_count / dt;
    double hw_fs = hw_bclk / (32 * num_slots);
    double sw_fs = ctx.total_frames / dt;
    double err_ppm = (hw_fs - sample_rate) * 1e6 / sample_rate;

    /* ANSI color codes for terminal output */
    #define C_GREEN  "\033[32m"
    #define C_YELLOW "\033[33m"
    #define C_RED    "\033[31m"
    #define C_RESET  "\033[0m"

    const char *color, *status;
    if (hw_fs >= sample_rate * 0.9999 && hw_fs <= sample_rate * 1.0001) {
        color = C_GREEN; status = "PASS";
    } else if (hw_fs >= sample_rate * 0.999 && hw_fs <= sample_rate * 1.001) {
        color = C_YELLOW; status = "OK";
    } else {
        color = C_RED; status = "FAIL";
    }

    printf("    HW: Fs=%.1f Hz (%+.0f ppm)  SW: %.0f Hz  [%s%s%s]\n",
           hw_fs, err_ppm, sw_fs, color, status, C_RESET);

    /* Clean up PCNT: delete channel first, then unit */
    pcnt_unit_disable(pcnt);
    pcnt_del_channel(s_pcnt_chan);
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

    /* Suppress the expected "input clock GPIO is set, use external clk src" warning */
    esp_log_level_set("parlio", ESP_LOG_ERROR);

    ESP_LOGI(TAG, "=== PARLIO I2S Throughput Sweep (PCNT-verified, %ds each) ===", TEST_SECONDS);
    ESP_LOGI(TAG, "  %-22s %5s     %s  %s", "Config", "Fs", "Ch", "BCLK");
    ESP_LOGI(TAG, "  ----------------------------------------------------------");

    /* --- 48 kHz family --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  -- 48 kHz --");
    run_test("Stereo",       48000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo x2",    48000, PARLIO_I2S_MODE_STANDARD, 2, 256);
    run_test("TDM4 x1",      48000, PARLIO_I2S_MODE_TDM4,     1, 256);
    run_test("TDM4 x2",      48000, PARLIO_I2S_MODE_TDM4,     2, 256);
    run_test("TDM8 x1",      48000, PARLIO_I2S_MODE_TDM8,     1, 512);
    run_test("TDM8 x2",      48000, PARLIO_I2S_MODE_TDM8,     2, 512);

    /* --- 96 kHz --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  -- 96 kHz --");
    run_test("Stereo",       96000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo x2",    96000, PARLIO_I2S_MODE_STANDARD, 2, 256);
    run_test("TDM4 x1",      96000, PARLIO_I2S_MODE_TDM4,     1, 256);
    run_test("TDM8 x1",      96000, PARLIO_I2S_MODE_TDM8,     1, 512);

    /* --- 192 kHz --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  -- 192 kHz --");
    run_test("Stereo",       192000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo x2",    192000, PARLIO_I2S_MODE_STANDARD, 2, 256);
    run_test("TDM4 x1",      192000, PARLIO_I2S_MODE_TDM4,     1, 256);

    /* --- 44.1 kHz family --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  -- 44.1 kHz --");
    run_test("Stereo",       44100, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("TDM8 x1",      44100, PARLIO_I2S_MODE_TDM8,     1, 512);

    /* --- 88.2 kHz --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  -- 88.2 kHz --");
    run_test("Stereo",       88200, PARLIO_I2S_MODE_STANDARD, 1, 256);

    /* --- Lower rates --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  -- Other --");
    run_test("Stereo 8kHz",    8000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo 16kHz",  16000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo 22.05k", 22050, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo 32kHz",  32000, PARLIO_I2S_MODE_STANDARD, 1, 256);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== All tests complete ===");
    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
