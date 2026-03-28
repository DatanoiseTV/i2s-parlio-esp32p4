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
#define PIN_DATA2      GPIO_NUM_5
#define PIN_DATA3      GPIO_NUM_32
#define PIN_DATA4      GPIO_NUM_33
#define PIN_DATA5      GPIO_NUM_2
#define PIN_DATA6      GPIO_NUM_3

#define TEST_SECONDS   5
#define FRAMES_PER_BUF 128
#define MAX_RESULTS    40

#define C_GREEN  "\033[32m"
#define C_YELLOW "\033[33m"
#define C_RED    "\033[31m"
#define C_BOLD   "\033[1m"
#define C_RESET  "\033[0m"

typedef struct {
    char label[28];
    uint32_t sample_rate;
    int channels;
    double hw_fs;
    double sw_fs;
    double err_ppm;
    const char *status;
} result_t;

static result_t results[MAX_RESULTS];
static int num_results = 0;

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
                ph[ch % 16] += inc[ch % 16];
                buf[base + ch] = (int32_t)ph[ch % 16];
            }
        }
        size_t written = 0;
        parlio_i2s_tx_write(ctx->tx, buf, FRAMES_PER_BUF, &written, 1000);
        ctx->total_frames += written;
    }

    free(buf);
    vTaskDelete(NULL);
}

static pcnt_channel_handle_t s_pcnt_chan;

static pcnt_unit_handle_t setup_pcnt(gpio_num_t bclk_gpio)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -1, .high_limit = 32767,
        .flags.accum_count = 1,
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num = bclk_gpio, .level_gpio_num = -1,
    };
    s_pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_cfg, &s_pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(s_pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    gpio_input_enable(bclk_gpio);
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, 32767));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    return unit;
}

static const gpio_num_t data_gpios[] = {
    PIN_DATA0, PIN_DATA1, PIN_DATA2, PIN_DATA3,
    PIN_DATA4, PIN_DATA5, PIN_DATA6
};

static void run_test(const char *label, uint32_t sample_rate,
                     parlio_i2s_mode_t mode, uint8_t num_data_lines,
                     uint16_t mclk_mult)
{
    uint8_t num_slots = (uint8_t)mode;
    int num_channels = num_data_lines * num_slots;
    uint32_t bclk = (uint32_t)32 * num_slots * sample_rate;

    printf("  %-26s %6"PRIu32" Hz  %3d ch  BCLK=%7.3f MHz  ",
           label, sample_rate, num_channels, bclk / 1e6f);
    fflush(stdout);

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
        .dma_buffer_count = 4,
        .frames_per_buffer = FRAMES_PER_BUF,
    };
    for (int i = 0; i < num_data_lines && i < 7; i++)
        cfg.data_gpios[i] = data_gpios[i];

    parlio_i2s_tx_handle_t tx = NULL;
    esp_err_t err = parlio_i2s_tx_new(&cfg, &tx);
    if (err != ESP_OK) {
        printf(C_RED "SKIP (%s)" C_RESET "\n", esp_err_to_name(err));
        if (num_results < MAX_RESULTS) {
            result_t *r = &results[num_results++];
            snprintf(r->label, sizeof(r->label), "%s", label);
            r->sample_rate = sample_rate;
            r->channels = num_channels;
            r->hw_fs = 0; r->sw_fs = 0; r->err_ppm = 0;
            r->status = "SKIP";
        }
        return;
    }
    parlio_i2s_tx_enable(tx);

    pcnt_unit_handle_t pcnt = setup_pcnt(PIN_BCLK);

    static ctx_t ctx;
    ctx.tx = tx;
    ctx.num_channels = num_channels;
    ctx.sample_rate = sample_rate;
    ctx.total_frames = 0;
    ctx.running = true;

    xTaskCreatePinnedToCore(audio_task, "audio", 16384, &ctx,
                            configMAX_PRIORITIES - 1, NULL, 1);

    pcnt_unit_clear_count(pcnt);
    pcnt_unit_start(pcnt);
    int64_t t_start = esp_timer_get_time();

    vTaskDelay(pdMS_TO_TICKS(TEST_SECONDS * 1000));

    pcnt_unit_stop(pcnt);
    int final_count = 0;
    pcnt_unit_get_count(pcnt, &final_count);
    int64_t t_end = esp_timer_get_time();
    double dt = (t_end - t_start) / 1e6;

    ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(500));

    double hw_bclk = final_count / dt;
    double hw_fs = hw_bclk / (32.0 * num_slots);
    double sw_fs = ctx.total_frames / dt;
    double err_ppm = (hw_fs - sample_rate) * 1e6 / sample_rate;

    const char *color, *status;
    if (hw_fs >= sample_rate * 0.9999 && hw_fs <= sample_rate * 1.0001) {
        color = C_GREEN; status = "PASS";
    } else if (hw_fs >= sample_rate * 0.999 && hw_fs <= sample_rate * 1.001) {
        color = C_YELLOW; status = "OK";
    } else if (hw_fs > 0) {
        color = C_RED; status = "FAIL";
    } else {
        color = C_RED; status = "NO CLK";
    }

    printf("Fs=%7.1f (%+5.0f ppm) SW=%7.0f  [%s%s%s]\n",
           hw_fs, err_ppm, sw_fs, color, status, C_RESET);

    if (num_results < MAX_RESULTS) {
        result_t *r = &results[num_results++];
        snprintf(r->label, sizeof(r->label), "%s", label);
        r->sample_rate = sample_rate;
        r->channels = num_channels;
        r->hw_fs = hw_fs;
        r->sw_fs = sw_fs;
        r->err_ppm = err_ppm;
        r->status = status;
    }

    pcnt_unit_disable(pcnt);
    pcnt_del_channel(s_pcnt_chan);
    pcnt_del_unit(pcnt);
    parlio_i2s_tx_disable(tx);
    parlio_i2s_tx_delete(tx);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void print_summary(void)
{
    printf("\n");
    printf(C_BOLD "=== EXECUTIVE SUMMARY ===" C_RESET "\n");
    printf("%-26s %7s  %3s  %9s  %7s  %6s  %s\n",
           "Configuration", "Fs", "Ch", "HW Fs", "Error", "SW Fs", "Status");
    printf("------------------------------------------------------------------------\n");

    int pass = 0, ok = 0, fail = 0, skip = 0;
    for (int i = 0; i < num_results; i++) {
        result_t *r = &results[i];
        const char *color = C_RESET;
        if (strcmp(r->status, "PASS") == 0) { color = C_GREEN; pass++; }
        else if (strcmp(r->status, "OK") == 0) { color = C_YELLOW; ok++; }
        else if (strcmp(r->status, "SKIP") == 0) { color = C_YELLOW; skip++; }
        else { color = C_RED; fail++; }

        if (r->hw_fs > 0) {
            printf("%-26s %6"PRIu32"  %3d  %9.1f  %+5.0fppm  %7.0f  %s%s%s\n",
                   r->label, r->sample_rate, r->channels,
                   r->hw_fs, r->err_ppm, r->sw_fs,
                   color, r->status, C_RESET);
        } else {
            printf("%-26s %6"PRIu32"  %3d  %9s  %7s  %7s  %s%s%s\n",
                   r->label, r->sample_rate, r->channels,
                   "--", "--", "--",
                   color, r->status, C_RESET);
        }
    }

    printf("------------------------------------------------------------------------\n");
    printf("Total: %d tests | ", num_results);
    if (pass > 0) printf(C_GREEN "%d PASS" C_RESET " ", pass);
    if (ok > 0)   printf(C_YELLOW "%d OK" C_RESET " ", ok);
    if (fail > 0) printf(C_RED "%d FAIL" C_RESET " ", fail);
    if (skip > 0) printf(C_YELLOW "%d SKIP" C_RESET " ", skip);
    printf("\n\n");
}

void app_main(void)
{
    esp_ldo_channel_handle_t ldo4 = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 4, .voltage_mv = 3300,
        .flags = { .adjustable = false },
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo4));

    esp_log_level_set("parlio", ESP_LOG_ERROR);
    esp_log_level_set("pcnt", ESP_LOG_ERROR);

    printf("\n" C_BOLD "=== PARLIO I2S Throughput Sweep (PCNT-verified, %ds/test) ===" C_RESET "\n", TEST_SECONDS);
    printf("  GPIOs: MCLK=%d BCLK=%d LRCK=%d DATA=%d,%d,%d,%d,%d,%d,%d\n\n",
           PIN_MCLK, PIN_BCLK, PIN_LRCK,
           PIN_DATA0, PIN_DATA1, PIN_DATA2, PIN_DATA3,
           PIN_DATA4, PIN_DATA5, PIN_DATA6);

    /* ---- 48 kHz ---- */
    printf(C_BOLD "  -- 48 kHz --" C_RESET "\n");
    run_test("Stereo x1",          48000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo x2",          48000, PARLIO_I2S_MODE_STANDARD, 2, 256);
    run_test("Stereo x4",          48000, PARLIO_I2S_MODE_STANDARD, 4, 256);
    run_test("Stereo x7",          48000, PARLIO_I2S_MODE_STANDARD, 7, 256);
    run_test("TDM4 x1 (4ch)",      48000, PARLIO_I2S_MODE_TDM4,    1, 256);
    run_test("TDM4 x2 (8ch)",      48000, PARLIO_I2S_MODE_TDM4,    2, 256);
    run_test("TDM4 x4 (16ch)",     48000, PARLIO_I2S_MODE_TDM4,    4, 256);
    run_test("TDM4 x7 (28ch)",     48000, PARLIO_I2S_MODE_TDM4,    7, 256);
    run_test("TDM8 x1 (8ch)",      48000, PARLIO_I2S_MODE_TDM8,    1, 512);
    run_test("TDM8 x2 (16ch)",     48000, PARLIO_I2S_MODE_TDM8,    2, 512);
    run_test("TDM8 x4 (32ch)",     48000, PARLIO_I2S_MODE_TDM8,    4, 512);
    run_test("TDM8 x7 (56ch)",     48000, PARLIO_I2S_MODE_TDM8,    7, 512);

    /* ---- 96 kHz ---- */
    printf("\n" C_BOLD "  -- 96 kHz --" C_RESET "\n");
    run_test("Stereo x1",          96000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo x4",          96000, PARLIO_I2S_MODE_STANDARD, 4, 256);
    run_test("TDM4 x1 (4ch)",      96000, PARLIO_I2S_MODE_TDM4,    1, 256);
    run_test("TDM4 x4 (16ch)",     96000, PARLIO_I2S_MODE_TDM4,    4, 256);
    run_test("TDM8 x1 (8ch)",      96000, PARLIO_I2S_MODE_TDM8,    1, 512);
    run_test("TDM8 x2 (16ch)",     96000, PARLIO_I2S_MODE_TDM8,    2, 512);

    /* ---- 192 kHz ---- */
    printf("\n" C_BOLD "  -- 192 kHz --" C_RESET "\n");
    run_test("Stereo x1",          192000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo x4",          192000, PARLIO_I2S_MODE_STANDARD, 4, 256);
    run_test("TDM4 x1 (4ch)",      192000, PARLIO_I2S_MODE_TDM4,    1, 256);
    run_test("TDM8 x1 (8ch)",      192000, PARLIO_I2S_MODE_TDM8,    1, 512);

    /* ---- 44.1 kHz family ---- */
    printf("\n" C_BOLD "  -- 44.1 kHz family --" C_RESET "\n");
    run_test("Stereo 44.1k",       44100, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("TDM8 x2 44.1k",      44100, PARLIO_I2S_MODE_TDM8,    2, 512);
    run_test("Stereo 88.2k",       88200, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo 176.4k",     176400, PARLIO_I2S_MODE_STANDARD, 1, 256);

    /* ---- Low sample rates ---- */
    printf("\n" C_BOLD "  -- Low rates --" C_RESET "\n");
    run_test("Stereo 8kHz",         8000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo 16kHz",       16000, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo 22.05kHz",    22050, PARLIO_I2S_MODE_STANDARD, 1, 256);
    run_test("Stereo 32kHz",       32000, PARLIO_I2S_MODE_STANDARD, 1, 256);

    /* ---- Summary ---- */
    print_summary();

    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
