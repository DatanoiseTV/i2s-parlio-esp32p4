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
#include "parlio_audio_tx.h"

static const char *TAG = "audio_test";

#define PIN_MCLK       GPIO_NUM_21
#define PIN_BCLK       GPIO_NUM_22
#define PIN_LRCK       GPIO_NUM_23
#define PIN_DATA0      GPIO_NUM_5
#define PIN_DATA1      GPIO_NUM_32
#define PIN_DATA2      GPIO_NUM_33
#define PIN_DATA3      GPIO_NUM_36
#define PIN_DATA4      GPIO_NUM_54
/* Safe green GPIOs from ESP32-P4-NANO pinout (no USB/UART/XTAL/I2C) */
#define PIN_DATA5      GPIO_NUM_45
#define PIN_DATA6      GPIO_NUM_46
#define PIN_DATA7      GPIO_NUM_47
#define PIN_DATA8      GPIO_NUM_48
#define PIN_DATA9      GPIO_NUM_6
#define PIN_DATA10     GPIO_NUM_53

#define TEST_SECONDS   3
#define FRAMES_PER_BUF 128
#define MAX_RESULTS    40
static int test_num = 0;
static int test_total = 0;

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
    int pcnt_counted;
    int pcnt_expected;
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
    PIN_DATA4, PIN_DATA5, PIN_DATA6, PIN_DATA7,
    PIN_DATA8, PIN_DATA9, PIN_DATA10
};
#define NUM_DATA_GPIOS (sizeof(data_gpios) / sizeof(data_gpios[0]))

static void run_test(const char *label, uint32_t sample_rate,
                     parlio_i2s_mode_t mode, uint8_t num_data_lines,
                     uint16_t mclk_mult)
{
    uint8_t num_slots = (uint8_t)mode;
    int num_channels = num_data_lines * num_slots;
    uint32_t bclk = (uint32_t)32 * num_slots * sample_rate;

    test_num++;
    printf("\n  " C_BOLD "[%2d/%d]" C_RESET " %-24s  %6"PRIu32" Hz  %3d ch  BCLK=%.3f MHz\n",
           test_num, test_total, label, sample_rate, num_channels, bclk / 1e6f);
    printf("         ");
    fflush(stdout);

    /* Adaptive frames_per_buffer: reduce for high channel counts to fit in RAM.
     * Each frame = bclk_per_frame bytes (pw=8) or *2 (pw=16).
     * 3 DMA buffers must fit in ~300KB. */
    uint32_t bclk_per_frame = 32 * num_slots;
    size_t bytes_per_frame = bclk_per_frame * (num_data_lines > 7 ? 2 : 1);
    size_t fpb = FRAMES_PER_BUF;
    while (fpb > 16 && fpb * bytes_per_frame * 3 > 250000)
        fpb /= 2;

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
        .frames_per_buffer = fpb,
    };
    for (int i = 0; i < num_data_lines && i < (int)NUM_DATA_GPIOS; i++)
        cfg.data_gpios[i] = data_gpios[i];

    parlio_i2s_tx_handle_t tx = NULL;
    esp_err_t err = parlio_i2s_tx_new(&cfg, &tx);
    if (err != ESP_OK) {
        uint32_t mclk_hz = mclk_mult * sample_rate;
        printf(C_YELLOW "SKIP" C_RESET " -- %s (MCLK=%.1f MHz too high?)\n",
               esp_err_to_name(err), mclk_hz / 1e6f);
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

    /* Progress dots during measurement */
    for (int s = 0; s < TEST_SECONDS; s++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf(".");
        fflush(stdout);
    }

    pcnt_unit_stop(pcnt);
    int final_count = 0;
    pcnt_unit_get_count(pcnt, &final_count);
    int64_t t_end = esp_timer_get_time();
    double dt = (t_end - t_start) / 1e6;

    ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(200));

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

    int pcnt_expected = (int)(bclk * dt + 0.5);
    printf(" %s%s%s  Fs=%.1f Hz (%+.0f ppm)  PCNT=%d/%d\n",
           color, status, C_RESET, hw_fs, err_ppm, final_count, pcnt_expected);

    if (num_results < MAX_RESULTS) {
        result_t *r = &results[num_results++];
        snprintf(r->label, sizeof(r->label), "%s", label);
        r->sample_rate = sample_rate;
        r->channels = num_channels;
        r->hw_fs = hw_fs;
        r->sw_fs = sw_fs;
        r->err_ppm = err_ppm;
        r->pcnt_counted = final_count;
        r->pcnt_expected = pcnt_expected;
        r->status = status;
    }

    pcnt_unit_disable(pcnt);
    pcnt_del_channel(s_pcnt_chan);
    pcnt_del_unit(pcnt);
    parlio_i2s_tx_disable(tx);
    parlio_i2s_tx_delete(tx);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* ------------------------------------------------------------------ */
/*  Unified driver test (ADAT, ADAT+I2S combos)                        */
/* ------------------------------------------------------------------ */

typedef struct {
    parlio_audio_tx_handle_t tx;
    size_t frame_size;
    uint32_t sample_rate;
    volatile uint32_t total_frames;
    volatile bool running;
} uctx_t;

static void unified_audio_task(void *arg)
{
    uctx_t *ctx = (uctx_t *)arg;
    const size_t fs = ctx->frame_size;

    int32_t *buf = malloc(FRAMES_PER_BUF * fs * sizeof(int32_t));
    if (!buf) { ctx->running = false; vTaskDelete(NULL); return; }

    uint32_t ph[16] = {0};
    uint32_t inc[16];
    for (int ch = 0; ch < (int)fs && ch < 16; ch++)
        inc[ch] = (uint32_t)(((uint64_t)(200 + ch * 100) << 32) / ctx->sample_rate);

    while (ctx->running) {
        for (size_t f = 0; f < FRAMES_PER_BUF; f++) {
            size_t base = f * fs;
            for (size_t ch = 0; ch < fs; ch++) {
                ph[ch % 16] += inc[ch % 16];
                buf[base + ch] = (int32_t)ph[ch % 16];
            }
        }
        size_t written = 0;
        parlio_audio_tx_write(ctx->tx, buf, FRAMES_PER_BUF, &written, 1000);
        ctx->total_frames += written;
    }

    free(buf);
    vTaskDelete(NULL);
}

static void run_unified_test(const char *label, uint32_t sample_rate,
                              const parlio_audio_i2s_config_t *i2s_cfg,
                              const parlio_audio_adat_config_t *adat_cfg,
                              uint16_t mclk_mult)
{
    /* Count total channels */
    int num_channels = 0;
    uint32_t ticks_per_frame = 0;
    if (i2s_cfg) {
        uint8_t ns = (uint8_t)(i2s_cfg->mode ? i2s_cfg->mode : 2);
        num_channels += i2s_cfg->num_data_lines * ns;
        uint32_t bclk_tpf = 32 * ns;
        if (bclk_tpf > ticks_per_frame) ticks_per_frame = bclk_tpf;
    }
    if (adat_cfg) {
        num_channels += 8;
        uint32_t adat_tpf = 256;
        if (adat_tpf > ticks_per_frame) ticks_per_frame = adat_tpf;
    }
    /* In multi-protocol mode, PARLIO clock = max protocol rate.
     * clk_out outputs that clock. ticks_per_frame = parlio_clock / Fs. */
    uint32_t parlio_clock = ticks_per_frame * sample_rate;

    test_num++;
    printf("\n  " C_BOLD "[%2d/%d]" C_RESET " %-24s  %6"PRIu32" Hz  %3d ch  CLK=%.3f MHz\n",
           test_num, test_total, label, sample_rate, num_channels, parlio_clock / 1e6f);
    printf("         ");
    fflush(stdout);

    parlio_audio_tx_config_t cfg = {
        .sample_rate = sample_rate,
        .mclk_multiple = mclk_mult,
        .mclk_gpio = PIN_MCLK,
        .clk_out_gpio = PIN_BCLK, /* PCNT measures this pin */
        .i2s = i2s_cfg,
        .adat = adat_cfg,
        .dma_buffer_count = 4,
        .frames_per_buffer = FRAMES_PER_BUF,
    };

    parlio_audio_tx_handle_t tx = NULL;
    esp_err_t err = parlio_audio_tx_new(&cfg, &tx);
    if (err != ESP_OK) {
        uint32_t mclk_hz = mclk_mult * sample_rate;
        printf(C_YELLOW "SKIP" C_RESET " -- %s (MCLK=%.1f MHz)\n",
               esp_err_to_name(err), mclk_hz / 1e6f);
        if (num_results < MAX_RESULTS) {
            result_t *r = &results[num_results++];
            snprintf(r->label, sizeof(r->label), "%s", label);
            r->sample_rate = sample_rate;
            r->channels = num_channels;
            r->status = "SKIP";
        }
        return;
    }
    parlio_audio_tx_enable(tx);

    pcnt_unit_handle_t pcnt = setup_pcnt(PIN_BCLK);

    static uctx_t uctx;
    uctx.tx = tx;
    uctx.frame_size = parlio_audio_tx_get_frame_size(tx);
    uctx.sample_rate = sample_rate;
    uctx.total_frames = 0;
    uctx.running = true;

    xTaskCreatePinnedToCore(unified_audio_task, "uaudio", 16384, &uctx,
                            configMAX_PRIORITIES - 1, NULL, 1);

    pcnt_unit_clear_count(pcnt);
    pcnt_unit_start(pcnt);
    int64_t t_start = esp_timer_get_time();

    for (int s = 0; s < TEST_SECONDS; s++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf(".");
        fflush(stdout);
    }

    pcnt_unit_stop(pcnt);
    int final_count = 0;
    pcnt_unit_get_count(pcnt, &final_count);
    int64_t t_end = esp_timer_get_time();
    double dt = (t_end - t_start) / 1e6;

    uctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(200));

    /* clk_out = PARLIO clock = ticks_per_frame * Fs.
     * Measured Fs = counted_edges / dt / ticks_per_frame */
    double hw_clk = final_count / dt;
    double hw_fs = hw_clk / ticks_per_frame;
    double sw_fs = uctx.total_frames / dt;
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

    int pcnt_expected = (int)(parlio_clock * dt + 0.5);
    printf(" %s%s%s  Fs=%.1f Hz (%+.0f ppm)  PCNT=%d/%d\n",
           color, status, C_RESET, hw_fs, err_ppm, final_count, pcnt_expected);

    if (num_results < MAX_RESULTS) {
        result_t *r = &results[num_results++];
        snprintf(r->label, sizeof(r->label), "%s", label);
        r->sample_rate = sample_rate;
        r->channels = num_channels;
        r->hw_fs = hw_fs;
        r->sw_fs = sw_fs;
        r->err_ppm = err_ppm;
        r->pcnt_counted = final_count;
        r->pcnt_expected = pcnt_expected;
        r->status = status;
    }

    pcnt_unit_disable(pcnt);
    pcnt_del_channel(s_pcnt_chan);
    pcnt_del_unit(pcnt);
    parlio_audio_tx_disable(tx);
    parlio_audio_tx_delete(tx);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void print_summary(void)
{
    printf("\n");
    printf(C_BOLD "========================================" C_RESET "\n");
    printf(C_BOLD "  EXECUTIVE SUMMARY" C_RESET "\n");
    printf(C_BOLD "========================================" C_RESET "\n\n");
    printf("  %-24s %7s  %3s  %10s  %7s  %15s  %s\n",
           "Configuration", "Target", "Ch", "Measured", "Error", "PCNT cnt/exp", "");
    printf("  -----------------------------------------------------------------------------------------\n");

    int pass = 0, ok = 0, fail = 0, skip = 0;
    for (int i = 0; i < num_results; i++) {
        result_t *r = &results[i];
        const char *color = C_RESET;
        if (strcmp(r->status, "PASS") == 0) { color = C_GREEN; pass++; }
        else if (strcmp(r->status, "OK") == 0) { color = C_YELLOW; ok++; }
        else if (strcmp(r->status, "SKIP") == 0) { color = C_YELLOW; skip++; }
        else { color = C_RED; fail++; }

        if (r->hw_fs > 0) {
            printf("  %-24s %6"PRIu32"  %3d  %9.1f Hz  %+5.0fppm  %7d/%-7d  %s%4s%s\n",
                   r->label, r->sample_rate, r->channels,
                   r->hw_fs, r->err_ppm,
                   r->pcnt_counted, r->pcnt_expected,
                   color, r->status, C_RESET);
        } else {
            printf("  %-24s %6"PRIu32"  %3d  %9s     %7s  %15s  %s%4s%s\n",
                   r->label, r->sample_rate, r->channels,
                   "--", "--", "--",
                   color, r->status, C_RESET);
        }
    }

    printf("  -----------------------------------------------------------------------------------------\n");
    printf("  %d tests: ", num_results);
    if (pass > 0) printf(C_GREEN "%d PASS" C_RESET "  ", pass);
    if (ok > 0)   printf(C_YELLOW "%d OK" C_RESET "  ", ok);
    if (fail > 0) printf(C_RED "%d FAIL" C_RESET "  ", fail);
    if (skip > 0) printf(C_YELLOW "%d SKIP" C_RESET "  ", skip);
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

    /* Count total tests */
    test_total = 17 + 5 + 3 + 4 + 3 + 9; /* 48k + 96k + 192k + 44.1k + low + unified */

    printf("\n" C_BOLD "========================================" C_RESET "\n");
    printf(C_BOLD "  PARLIO I2S Throughput Sweep" C_RESET "\n");
    printf(C_BOLD "  PCNT-verified, %ds per test" C_RESET "\n", TEST_SECONDS);
    printf(C_BOLD "========================================" C_RESET "\n\n");
    printf("  MCLK=GPIO%d  BCLK=GPIO%d  LRCK=GPIO%d\n", PIN_MCLK, PIN_BCLK, PIN_LRCK);
    printf("  DATA[0..10]: GPIO %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
           PIN_DATA0, PIN_DATA1, PIN_DATA2, PIN_DATA3, PIN_DATA4,
           PIN_DATA5, PIN_DATA6, PIN_DATA7, PIN_DATA8, PIN_DATA9, PIN_DATA10);
    printf("  11 data lines = max 176 channels (TDM16 x 11)\n");

    /* ---- 48 kHz: scaling data lines ---- */
    printf("\n" C_BOLD "  --- 48 kHz: channel scaling ---" C_RESET "\n");
    run_test("Stereo x1 (2ch)",     48000, PARLIO_I2S_MODE_STANDARD, 1,  256);
    run_test("Stereo x4 (8ch)",     48000, PARLIO_I2S_MODE_STANDARD, 4,  256);
    run_test("Stereo x7 (14ch)",    48000, PARLIO_I2S_MODE_STANDARD, 7,  256);
    run_test("Stereo x11 (22ch)",   48000, PARLIO_I2S_MODE_STANDARD, 11, 256);
    run_test("TDM4 x1 (4ch)",       48000, PARLIO_I2S_MODE_TDM4,    1,  256);
    run_test("TDM4 x4 (16ch)",      48000, PARLIO_I2S_MODE_TDM4,    4,  256);
    run_test("TDM4 x7 (28ch)",      48000, PARLIO_I2S_MODE_TDM4,    7,  256);
    run_test("TDM4 x11 (44ch)",     48000, PARLIO_I2S_MODE_TDM4,    11, 256);
    run_test("TDM8 x1 (8ch)",       48000, PARLIO_I2S_MODE_TDM8,    1,  512);
    run_test("TDM8 x2 (16ch)",      48000, PARLIO_I2S_MODE_TDM8,    2,  512);
    run_test("TDM8 x4 (32ch)",      48000, PARLIO_I2S_MODE_TDM8,    4,  512);
    run_test("TDM8 x7 (56ch)",      48000, PARLIO_I2S_MODE_TDM8,    7,  512);
    run_test("TDM8 x11 (88ch)",     48000, PARLIO_I2S_MODE_TDM8,    11, 512);
    run_test("TDM16 x1 (16ch)",     48000, PARLIO_I2S_MODE_TDM16,   1,  1024);
    run_test("TDM16 x4 (64ch)",     48000, PARLIO_I2S_MODE_TDM16,   4,  1024);
    run_test("TDM16 x7 (112ch)",    48000, PARLIO_I2S_MODE_TDM16,   7,  1024);
    run_test("TDM16 x11 (176ch)",   48000, PARLIO_I2S_MODE_TDM16,   11, 1024);

    /* ---- 96 kHz ---- */
    printf("\n" C_BOLD "  --- 96 kHz ---" C_RESET "\n");
    run_test("Stereo x1 (2ch)",     96000, PARLIO_I2S_MODE_STANDARD, 1,  256);
    run_test("Stereo x11 (22ch)",   96000, PARLIO_I2S_MODE_STANDARD, 11, 256);
    run_test("TDM4 x4 (16ch)",      96000, PARLIO_I2S_MODE_TDM4,    4,  256);
    run_test("TDM8 x1 (8ch)",       96000, PARLIO_I2S_MODE_TDM8,    1,  256);
    run_test("TDM8 x4 (32ch)",      96000, PARLIO_I2S_MODE_TDM8,    4,  256);

    /* ---- 192 kHz ---- */
    printf("\n" C_BOLD "  --- 192 kHz ---" C_RESET "\n");
    run_test("Stereo x1 (2ch)",     192000, PARLIO_I2S_MODE_STANDARD, 1,  128);
    run_test("Stereo x11 (22ch)",   192000, PARLIO_I2S_MODE_STANDARD, 11, 128);
    run_test("TDM4 x1 (4ch)",       192000, PARLIO_I2S_MODE_TDM4,    1,  128);

    /* ---- 44.1 kHz family ---- */
    printf("\n" C_BOLD "  --- 44.1 kHz family ---" C_RESET "\n");
    run_test("Stereo 44.1k",        44100, PARLIO_I2S_MODE_STANDARD, 1,  256);
    run_test("TDM8 x4 44.1k (32ch)",44100, PARLIO_I2S_MODE_TDM8,    4,  512);
    run_test("Stereo 88.2k",        88200, PARLIO_I2S_MODE_STANDARD, 1,  256);
    run_test("Stereo 176.4k",      176400, PARLIO_I2S_MODE_STANDARD, 1,  256);

    /* ---- Low sample rates ---- */
    printf("\n" C_BOLD "  --- Low sample rates ---" C_RESET "\n");
    run_test("Stereo 8kHz",          8000, PARLIO_I2S_MODE_STANDARD, 1,  256);
    run_test("Stereo 16kHz",        16000, PARLIO_I2S_MODE_STANDARD, 1,  256);
    run_test("Stereo 32kHz",        32000, PARLIO_I2S_MODE_STANDARD, 1,  256);

    /* ---- Unified driver: ADAT and ADAT+I2S combos ---- */
    printf("\n" C_BOLD "  --- Unified driver: ADAT + I2S combos ---" C_RESET "\n");
    {
        /* ADAT only (8 ch) */
        parlio_audio_adat_config_t adat_only = { .adat_gpio = PIN_DATA0 };
        run_unified_test("ADAT only (8ch)", 48000, NULL, &adat_only, 512);

        /* ADAT + I2S stereo (10 ch) */
        parlio_audio_i2s_config_t i2s_1x2 = {
            .mode = PARLIO_AUDIO_I2S_STANDARD,
            .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 1,
            .bclk_gpio = PIN_DATA1, .lrck_gpio = PIN_DATA2,
            .data_gpios = { PIN_DATA3 },
        };
        parlio_audio_adat_config_t adat_w_i2s = { .adat_gpio = PIN_DATA4 };
        run_unified_test("ADAT+Stereo (10ch)", 48000, &i2s_1x2, &adat_w_i2s, 512);

        /* ADAT + I2S TDM4 x2 (16 ch) */
        parlio_audio_i2s_config_t i2s_2x4 = {
            .mode = PARLIO_AUDIO_I2S_TDM4,
            .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 2,
            .bclk_gpio = PIN_DATA1, .lrck_gpio = PIN_DATA2,
            .data_gpios = { PIN_DATA3, PIN_DATA4 },
        };
        parlio_audio_adat_config_t adat_w_tdm4 = { .adat_gpio = PIN_DATA5 };
        run_unified_test("ADAT+TDM4x2 (16ch)", 48000, &i2s_2x4, &adat_w_tdm4, 512);

        /* ADAT + I2S TDM8 x1 (16 ch) */
        parlio_audio_i2s_config_t i2s_1x8 = {
            .mode = PARLIO_AUDIO_I2S_TDM8,
            .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 1,
            .bclk_gpio = PIN_DATA1, .lrck_gpio = PIN_DATA2,
            .data_gpios = { PIN_DATA3 },
        };
        parlio_audio_adat_config_t adat_w_tdm8 = { .adat_gpio = PIN_DATA4 };
        run_unified_test("ADAT+TDM8x1 (16ch)", 48000, &i2s_1x8, &adat_w_tdm8, 512);

        /* ADAT + I2S stereo x4 (16 ch) */
        parlio_audio_i2s_config_t i2s_4x2 = {
            .mode = PARLIO_AUDIO_I2S_STANDARD,
            .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 4,
            .bclk_gpio = PIN_DATA1, .lrck_gpio = PIN_DATA2,
            .data_gpios = { PIN_DATA3, PIN_DATA4, PIN_DATA5, PIN_DATA6 },
        };
        parlio_audio_adat_config_t adat_w_4stereo = { .adat_gpio = PIN_DATA7 };
        run_unified_test("ADAT+Stereo4 (16ch)", 48000, &i2s_4x2, &adat_w_4stereo, 512);

        /* ADAT + I2S TDM8 x4 (40 ch) */
        parlio_audio_i2s_config_t i2s_4x8 = {
            .mode = PARLIO_AUDIO_I2S_TDM8,
            .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 4,
            .bclk_gpio = PIN_DATA1, .lrck_gpio = PIN_DATA2,
            .data_gpios = { PIN_DATA3, PIN_DATA4, PIN_DATA5, PIN_DATA6 },
        };
        parlio_audio_adat_config_t adat_w_tdm8x4 = { .adat_gpio = PIN_DATA7 };
        run_unified_test("ADAT+TDM8x4 (40ch)", 48000, &i2s_4x8, &adat_w_tdm8x4, 512);

        /* ADAT + I2S TDM4 x7 (36 ch) */
        parlio_audio_i2s_config_t i2s_7x4 = {
            .mode = PARLIO_AUDIO_I2S_TDM4,
            .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 7,
            .bclk_gpio = PIN_DATA1, .lrck_gpio = PIN_DATA2,
            .data_gpios = { PIN_DATA3, PIN_DATA4, PIN_DATA5, PIN_DATA6,
                            PIN_DATA7, PIN_DATA8, PIN_DATA9 },
        };
        parlio_audio_adat_config_t adat_w_tdm4x7 = { .adat_gpio = PIN_DATA10 };
        run_unified_test("ADAT+TDM4x7 (36ch)", 48000, &i2s_7x4, &adat_w_tdm4x7, 512);

        /* ADAT + I2S Stereo x8 (24 ch) */
        parlio_audio_i2s_config_t i2s_8x2 = {
            .mode = PARLIO_AUDIO_I2S_STANDARD,
            .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 8,
            .bclk_gpio = PIN_DATA1, .lrck_gpio = PIN_DATA2,
            .data_gpios = { PIN_DATA3, PIN_DATA4, PIN_DATA5, PIN_DATA6,
                            PIN_DATA7, PIN_DATA8, PIN_DATA9, PIN_DATA10 },
        };
        parlio_audio_adat_config_t adat_w_8stereo = { .adat_gpio = PIN_LRCK };
        /* Note: reuse LRCK pin for ADAT since unified driver assigns its own TXD */
        run_unified_test("ADAT+Stereo8 (24ch)", 48000, &i2s_8x2, &adat_w_8stereo, 512);

        /* ADAT 44.1 kHz (8 ch) */
        parlio_audio_adat_config_t adat_441 = { .adat_gpio = PIN_DATA0 };
        run_unified_test("ADAT 44.1k (8ch)", 44100, NULL, &adat_441, 512);
    }

    /* ---- Summary ---- */
    print_summary();

    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
