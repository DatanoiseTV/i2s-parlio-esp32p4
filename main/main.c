#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/parlio_tx.h"
#include "esp_ldo_regulator.h"
#include "soc/gpio_sig_map.h"
#include "soc/parlio_periph.h"

static const char *TAG = "parlio_i2s_test";

/*
 * Test: verify PARLIO-based I2S TX output by reading back GPIO states.
 *
 * Physical wire: GPIO 20 (I2S MCLK out) -> GPIO 21 (PARLIO ext clk in).
 *
 * PARLIO outputs:
 *   clk_out (BCLK) = GPIO 22
 *   TXD[0]  (LRCK) = GPIO 23
 *   TXD[1]  (DATA) = GPIO 24
 *
 * We transmit a known pattern and sample GPIO states in a tight loop
 * to verify BCLK toggles, LRCK frames correctly, and DATA carries bits.
 */

#define PIN_MCLK_OUT   GPIO_NUM_20
#define PIN_PARLIO_CLK GPIO_NUM_21
#define PIN_BCLK       GPIO_NUM_22
#define PIN_LRCK       GPIO_NUM_23
#define PIN_DATA       GPIO_NUM_24

#define SAMPLE_RATE    48000
#define BITS           32
#define SLOT_WIDTH     32
#define MCLK_MULT      256

#include "driver/i2s_std.h"

void app_main(void)
{
    /* Power GPIO bank */
    esp_ldo_channel_handle_t ldo4 = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 4, .voltage_mv = 3300,
        .flags = { .adjustable = false },
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo4));
    ESP_LOGI(TAG, "LDO4 enabled at 3.3V");

    /* --- Set up I2S MCLK from APLL on GPIO 20 --- */
    i2s_chan_handle_t i2s_clk = NULL;
    {
        i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
        cc.dma_desc_num = 2; cc.dma_frame_num = 16;
        ESP_ERROR_CHECK(i2s_new_channel(&cc, &i2s_clk, NULL));
        i2s_std_config_t sc = {
            .clk_cfg = { .sample_rate_hz = SAMPLE_RATE, .clk_src = I2S_CLK_SRC_APLL,
                          .mclk_multiple = (i2s_mclk_multiple_t)MCLK_MULT },
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = { .mclk = PIN_MCLK_OUT, .bclk = I2S_GPIO_UNUSED,
                          .ws = I2S_GPIO_UNUSED, .dout = I2S_GPIO_UNUSED, .din = I2S_GPIO_UNUSED },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_clk, &sc));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_clk));
    }
    uint32_t mclk_freq = MCLK_MULT * SAMPLE_RATE;
    uint32_t bclk_freq = SLOT_WIDTH * 2 * SAMPLE_RATE;
    ESP_LOGI(TAG, "MCLK=%"PRIu32" Hz on GPIO %d, target BCLK=%"PRIu32" Hz",
             mclk_freq, PIN_MCLK_OUT, bclk_freq);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* --- Set up PARLIO TX --- */
    parlio_tx_unit_handle_t parlio = NULL;
    {
        parlio_tx_unit_config_t pc = {
            .clk_src = PARLIO_CLK_SRC_DEFAULT,
            .clk_in_gpio_num = PIN_PARLIO_CLK,
            .input_clk_src_freq_hz = mclk_freq,
            .output_clk_freq_hz = bclk_freq,
            .data_width = 2,
            .clk_out_gpio_num = PIN_BCLK,
            .valid_gpio_num = -1,
            .trans_queue_depth = 4,
            .max_transfer_size = 4096,
            .sample_edge = PARLIO_SAMPLE_EDGE_POS,
            .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
            .flags = { .clk_gate_en = 0, .io_loop_back = 1 },
        };
        for (int i = 0; i < 16; i++) pc.data_gpio_nums[i] = -1;
        pc.data_gpio_nums[0] = PIN_LRCK;
        pc.data_gpio_nums[1] = PIN_DATA;
        ESP_ERROR_CHECK(parlio_new_tx_unit(&pc, &parlio));
        ESP_ERROR_CHECK(parlio_tx_unit_enable(parlio));
    }
    ESP_LOGI(TAG, "PARLIO TX enabled: BCLK=%d, LRCK=%d, DATA=%d", PIN_BCLK, PIN_LRCK, PIN_DATA);

    /* --- Encode a test pattern: one frame with L=0x80000000, R=0x7FFFFFFF ---
     * Left = MSB set (negative full scale), Right = MSB clear (positive full scale).
     * This means DATA toggles between 1 (left) and 0 (right) on the MSB position.
     * Easy to see on a scope. */
    const uint16_t bclk_per_frame = SLOT_WIDTH * 2;
    const size_t num_test_frames = 16;
    const size_t total_bclk = bclk_per_frame * num_test_frames;
    const size_t buf_bytes = total_bclk / 4; /* 4 x 2-bit words per byte */

    uint8_t *tx_buf = heap_caps_calloc(1, buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!tx_buf) { ESP_LOGE(TAG, "alloc failed"); return; }

    /* Encode frames: alternating +full/-full scale per frame for visibility */
    for (size_t f = 0; f < num_test_frames; f++) {
        int32_t left  = (f % 2 == 0) ? (int32_t)0x80000000 : (int32_t)0x7FFFFFFF;
        int32_t right = (f % 2 == 0) ? (int32_t)0x7FFFFFFF : (int32_t)0x80000000;

        for (uint16_t bclk = 0; bclk < bclk_per_frame; bclk++) {
            uint8_t slot = bclk / SLOT_WIDTH;
            uint16_t pos = bclk % SLOT_WIDTH;
            int16_t bit_idx = (int16_t)pos - 1;

            uint8_t lrck = (slot >= 1) ? 1 : 0;
            uint8_t data_bit = 0;
            if (bit_idx >= 0 && bit_idx < BITS) {
                int32_t samp = (slot == 0) ? left : right;
                data_bit = (samp >> (31 - bit_idx)) & 1;
            }

            uint8_t word = (lrck & 1) | ((data_bit & 1) << 1);
            size_t idx = f * bclk_per_frame + bclk;
            size_t byte_pos = idx / 4;
            size_t bit_pos  = (idx % 4) * 2;
            tx_buf[byte_pos] |= (word & 0x03) << bit_pos;
        }
    }

    ESP_LOGI(TAG, "Encoded %zu frames (%zu bytes)", num_test_frames, buf_bytes);
    ESP_LOGI(TAG, "First 16 bytes: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x",
             tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
             tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7],
             tx_buf[8], tx_buf[9], tx_buf[10], tx_buf[11],
             tx_buf[12], tx_buf[13], tx_buf[14], tx_buf[15]);

    /* --- Transmit and sample GPIOs to verify output --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Transmitting and sampling GPIO states ===");

    /* Enable GPIO input on the output pins for readback */
    gpio_set_direction(PIN_BCLK, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_LRCK, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_DATA, GPIO_MODE_INPUT);

    /* Re-attach PARLIO outputs (gpio_set_direction disconnected them) */
    esp_rom_gpio_connect_out_signal(PIN_BCLK, PARLIO_TX_CLK_PAD_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(PIN_LRCK, PARLIO_TX_DATA0_PAD_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(PIN_DATA, PARLIO_TX_DATA1_PAD_OUT_IDX, false, false);

    /* Sample GPIO states during transmission.
     * At 3.072 MHz BCLK, each cycle is ~325 ns. CPU at 360 MHz can sample
     * ~117 cycles per BCLK, so we'll catch plenty of transitions. */

    #define NUM_SAMPLES 256
    uint8_t bclk_samples[NUM_SAMPLES];
    uint8_t lrck_samples[NUM_SAMPLES];
    uint8_t data_samples[NUM_SAMPLES];

    /* Start transmission */
    parlio_transmit_config_t tcfg = { .idle_value = 0 };
    ESP_ERROR_CHECK(parlio_tx_unit_transmit(parlio, tx_buf, total_bclk * 2, &tcfg));

    /* Tight sampling loop using gpio_get_level.
     * At 360 MHz CPU vs 3 MHz BCLK, each BCLK cycle is ~120 CPU cycles.
     * With the delay we sample roughly every 2-3 BCLK cycles. */
    for (int i = 0; i < NUM_SAMPLES; i++) {
        bclk_samples[i] = gpio_get_level(PIN_BCLK);
        lrck_samples[i] = gpio_get_level(PIN_LRCK);
        data_samples[i] = gpio_get_level(PIN_DATA);
        for (volatile int d = 0; d < 30; d++) {}
    }

    parlio_tx_unit_wait_all_done(parlio, 1000);

    /* Print sampled GPIO states */
    ESP_LOGI(TAG, "GPIO samples (B=BCLK, L=LRCK, D=DATA):");
    for (int row = 0; row < NUM_SAMPLES / 32; row++) {
        char line[200];
        int pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "  B: ");
        for (int i = 0; i < 32; i++)
            pos += snprintf(line + pos, sizeof(line) - pos, "%d", bclk_samples[row * 32 + i]);
        ESP_LOGI(TAG, "%s", line);

        pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "  L: ");
        for (int i = 0; i < 32; i++)
            pos += snprintf(line + pos, sizeof(line) - pos, "%d", lrck_samples[row * 32 + i]);
        ESP_LOGI(TAG, "%s", line);

        pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "  D: ");
        for (int i = 0; i < 32; i++)
            pos += snprintf(line + pos, sizeof(line) - pos, "%d", data_samples[row * 32 + i]);
        ESP_LOGI(TAG, "%s", line);
        ESP_LOGI(TAG, "");
    }

    /* Count transitions */
    int bclk_transitions = 0, lrck_transitions = 0, data_transitions = 0;
    for (int i = 1; i < NUM_SAMPLES; i++) {
        if (bclk_samples[i] != bclk_samples[i-1]) bclk_transitions++;
        if (lrck_samples[i] != lrck_samples[i-1]) lrck_transitions++;
        if (data_samples[i] != data_samples[i-1]) data_transitions++;
    }
    ESP_LOGI(TAG, "Transitions in %d samples: BCLK=%d, LRCK=%d, DATA=%d",
             NUM_SAMPLES, bclk_transitions, lrck_transitions, data_transitions);

    if (bclk_transitions > 10 && lrck_transitions > 0 && data_transitions > 0) {
        ESP_LOGI(TAG, "PASS -- all signals toggling");
    } else if (bclk_transitions > 10 && lrck_transitions > 0) {
        ESP_LOGW(TAG, "PARTIAL -- BCLK and LRCK active, DATA not toggling");
    } else if (bclk_transitions > 10) {
        ESP_LOGW(TAG, "PARTIAL -- only BCLK active");
    } else {
        ESP_LOGE(TAG, "FAIL -- no signal activity (check wire GPIO %d -> %d)",
                 PIN_MCLK_OUT, PIN_PARLIO_CLK);
    }

    /* Keep transmitting for scope/analyzer inspection */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Now looping continuous output for scope/analyzer verification...");
    while (1) {
        memset(tx_buf, 0, buf_bytes);
        for (size_t f = 0; f < num_test_frames; f++) {
            int32_t left  = (f % 2 == 0) ? (int32_t)0x80000000 : (int32_t)0x7FFFFFFF;
            int32_t right = (f % 2 == 0) ? (int32_t)0x7FFFFFFF : (int32_t)0x80000000;
            for (uint16_t bclk = 0; bclk < bclk_per_frame; bclk++) {
                uint8_t slot = bclk / SLOT_WIDTH;
                uint16_t pos = bclk % SLOT_WIDTH;
                int16_t bit_idx = (int16_t)pos - 1;
                uint8_t lrck = (slot >= 1) ? 1 : 0;
                uint8_t data_bit = 0;
                if (bit_idx >= 0 && bit_idx < BITS) {
                    int32_t samp = (slot == 0) ? left : right;
                    data_bit = (samp >> (31 - bit_idx)) & 1;
                }
                uint8_t word = (lrck & 1) | ((data_bit & 1) << 1);
                size_t idx = f * bclk_per_frame + bclk;
                tx_buf[idx / 4] |= (word & 0x03) << ((idx % 4) * 2);
            }
        }
        parlio_tx_unit_transmit(parlio, tx_buf, total_bclk * 2, &tcfg);
        parlio_tx_unit_wait_all_done(parlio, 1000);
    }
}
