#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/i2s_std.h"
#include "driver/parlio_tx.h"
#include "parlio_i2s.h"

static const char *TAG = "loopback_test";

/*
 * Loopback test: PARLIO TX -> I2S RX
 *
 * Validates the full signal chain:
 *   APLL -> I2S MCLK (GPIO 20) -> [wire] -> PARLIO ext clk (GPIO 21)
 *   PARLIO clk_out (BCLK) -> I2S RX BCLK
 *   PARLIO TXD[0] (LRCK)  -> I2S RX WS
 *   PARLIO TXD[1] (DATA)  -> I2S RX DIN
 *
 * The I2S RX runs in slave mode, clocked by PARLIO's BCLK/LRCK output.
 * GPIO matrix routes the PARLIO output pins to I2S RX inputs internally.
 *
 * Test sends known patterns, reads them back, and compares.
 */

/* Physical wire: GPIO 20 (I2S MCLK out) -> GPIO 21 (PARLIO ext clk in) */
#define PIN_MCLK_OUT   GPIO_NUM_20   /* I2S MCLK output + DAC clock */
#define PIN_PARLIO_CLK GPIO_NUM_21   /* PARLIO ext clock input (wired to MCLK_OUT) */

/* PARLIO TX outputs -- I2S RX reads these via GPIO matrix */
#define PIN_BCLK       GPIO_NUM_22   /* PARLIO clk_out = BCLK */
#define PIN_LRCK       GPIO_NUM_23   /* PARLIO TXD[0] = LRCK / WS */
#define PIN_DATA       GPIO_NUM_24   /* PARLIO TXD[1] = audio data */

#define SAMPLE_RATE    48000
#define BITS           32
#define SLOT_WIDTH     32
#define MCLK_MULT      256   /* MCLK/BCLK = 256/64 = 4 */

/* Number of frames to test per iteration */
#define TEST_FRAMES    64

/* Test pattern: alternating ramp values that are easy to verify.
 * Left channel = frame index * 0x01000000 (ramp in upper byte)
 * Right channel = bitwise inverse of left */
static void generate_test_pattern(int32_t *buf, size_t num_frames, uint32_t start_idx)
{
    for (size_t f = 0; f < num_frames; f++) {
        uint32_t idx = (start_idx + f) & 0xFF;
        int32_t left  = (int32_t)(idx << 24);   /* ramp in MSByte */
        int32_t right = ~left;                    /* inverse */
        buf[f * 2 + 0] = left;
        buf[f * 2 + 1] = right;
    }
}

/*
 * Set up I2S RX in slave mode to capture data from PARLIO TX.
 *
 * In slave mode, BCLK and WS are inputs. We route them from the same
 * GPIO pins that PARLIO drives, via the GPIO matrix. The I2S peripheral
 * reads the pin state, so it sees what PARLIO outputs.
 */
static esp_err_t setup_i2s_rx(i2s_chan_handle_t *rx_chan)
{
    /* Use I2S_NUM_1 since I2S_NUM_0 is occupied by the MCLK clock generator */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = TEST_FRAMES;

    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&chan_cfg, NULL, rx_chan),
        TAG, "failed to create I2S RX channel");

    /* In slave mode, use external clock source and provide the BCLK frequency.
     * BCLK comes from PARLIO clk_out, WS from PARLIO TXD[0]. */
    uint32_t bclk_freq = SLOT_WIDTH * 2 * SAMPLE_RATE; /* 3.072 MHz for 48k/32bit stereo */
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_EXTERNAL,
            .ext_clk_freq_hz = bclk_freq,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = PIN_BCLK,         /* "MCLK" input = BCLK pin (external clock source) */
            .bclk = PIN_BCLK,          /* reads BCLK from PARLIO clk_out */
            .ws   = PIN_LRCK,          /* reads WS from PARLIO TXD[0] */
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_DATA,           /* reads data from PARLIO TXD[1] */
        },
    };

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(*rx_chan, &std_cfg),
        TAG, "failed to init I2S RX STD mode");

    return ESP_OK;
}

/*
 * Compare TX and RX buffers, accounting for the pipeline delay.
 * Returns the number of matching frames found (after alignment).
 */
static int compare_buffers(const int32_t *tx_buf, size_t tx_frames,
                           const int32_t *rx_buf, size_t rx_frames)
{
    /* The I2S RX may have a few frames of latency. Search for the start
     * of the TX pattern within the RX buffer. */
    int best_offset = -1;
    int best_match = 0;

    for (size_t offset = 0; offset < rx_frames; offset++) {
        int matches = 0;
        size_t compare_len = (tx_frames < (rx_frames - offset))
                           ? tx_frames : (rx_frames - offset);

        for (size_t f = 0; f < compare_len; f++) {
            /* Compare left channel (masking to bits_per_sample precision).
             * RX may zero-pad LSBs differently, so compare upper 24 bits. */
            int32_t tx_l = tx_buf[f * 2 + 0] & (int32_t)0xFFFFFF00;
            int32_t rx_l = rx_buf[(offset + f) * 2 + 0] & (int32_t)0xFFFFFF00;
            int32_t tx_r = tx_buf[f * 2 + 1] & (int32_t)0xFFFFFF00;
            int32_t rx_r = rx_buf[(offset + f) * 2 + 1] & (int32_t)0xFFFFFF00;

            if (tx_l == rx_l && tx_r == rx_r) {
                matches++;
            }
        }

        if (matches > best_match) {
            best_match = matches;
            best_offset = (int)offset;
        }
    }

    if (best_offset >= 0 && best_match > 0) {
        ESP_LOGI(TAG, "best alignment at RX offset %d: %d/%zu frames match",
                 best_offset, best_match,
                 (tx_frames < (rx_frames - best_offset))
                    ? tx_frames : (rx_frames - best_offset));
    }
    return best_match;
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== PARLIO I2S TX -> I2S RX loopback test ===");
    ESP_LOGI(TAG, "Physical wire: GPIO %d (MCLK out) -> GPIO %d (PARLIO clk in)",
             PIN_MCLK_OUT, PIN_PARLIO_CLK);
    ESP_LOGI(TAG, "Internal loopback via GPIO matrix: BCLK=%d, LRCK=%d, DATA=%d",
             PIN_BCLK, PIN_LRCK, PIN_DATA);
    ESP_LOGI(TAG, "Fs=%d, %d-bit, MCLK=%dx", SAMPLE_RATE, BITS, MCLK_MULT);

    /* --- Set up PARLIO TX (our component) --- */
    parlio_i2s_tx_config_t tx_cfg = {
        .sample_rate    = SAMPLE_RATE,
        .bits_per_sample = BITS,
        .slot_width     = SLOT_WIDTH,
        .num_data_lines = 1,
        .mclk_multiple  = MCLK_MULT,
        .mode           = PARLIO_I2S_MODE_STANDARD,
        .mclk_gpio      = PIN_PARLIO_CLK,  /* PARLIO reads MCLK from this pin */
        .bclk_gpio      = PIN_BCLK,
        .lrck_gpio      = PIN_LRCK,
        .data_gpios     = { PIN_DATA },
        .dma_buffer_count  = 4,
        .frames_per_buffer = TEST_FRAMES,
    };

    /*
     * We need to set up the I2S MCLK output separately here because our
     * component's mclk_gpio is the PARLIO input pin (GPIO 21), but the
     * actual I2S MCLK output goes to GPIO 20. The physical wire bridges them.
     *
     * So we set up the APLL/I2S clock manually, then create the PARLIO TX
     * without the built-in I2S clock setup.
     *
     * Actually, let's keep it simpler: set mclk_gpio = PIN_PARLIO_CLK (21)
     * in our component. The component internally creates an I2S channel with
     * MCLK output on that pin. But we need MCLK on GPIO 20 too (the other
     * end of the wire). So we'll set up a second I2S channel just for MCLK
     * output on GPIO 20, or rely on the wire to carry the signal.
     *
     * Wait -- the component outputs I2S MCLK on mclk_gpio AND uses it as
     * PARLIO ext clk input. With the physical wire, we want:
     *   - I2S MCLK out on GPIO 20
     *   - Wire to GPIO 21
     *   - PARLIO reads ext clk from GPIO 21
     *
     * So we need to split: I2S MCLK on GPIO 20, PARLIO ext clk on GPIO 21.
     * Let me set up I2S MCLK manually and then create PARLIO without the
     * component's built-in I2S setup.
     *
     * The simplest approach: just use a slightly modified config. Our
     * component puts I2S MCLK on mclk_gpio. If we set mclk_gpio = GPIO 20,
     * the I2S MCLK goes to GPIO 20 (wire) -> GPIO 21 -> PARLIO. But PARLIO
     * needs to know its ext clock input is GPIO 21, not GPIO 20.
     *
     * Hmm, the component currently uses mclk_gpio for both I2S output and
     * PARLIO input. For the wire test we need them on different pins.
     * Let me just do it manually.
     */

    /* --- Step 1: Set up I2S MCLK on GPIO 20 (the "output" end of the wire) --- */
    ESP_LOGI(TAG, "Setting up APLL/I2S MCLK on GPIO %d...", PIN_MCLK_OUT);
    i2s_chan_handle_t i2s_clk_chan = NULL;
    {
        i2s_chan_config_t clk_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
        clk_cfg.dma_desc_num = 2;
        clk_cfg.dma_frame_num = 16;
        ESP_ERROR_CHECK(i2s_new_channel(&clk_cfg, &i2s_clk_chan, NULL));

        i2s_std_config_t std_cfg = {
            .clk_cfg = {
                .sample_rate_hz = SAMPLE_RATE,
                .clk_src = I2S_CLK_SRC_APLL,
                .mclk_multiple = (i2s_mclk_multiple_t)MCLK_MULT,
            },
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = PIN_MCLK_OUT,
                .bclk = I2S_GPIO_UNUSED,
                .ws   = I2S_GPIO_UNUSED,
                .dout = I2S_GPIO_UNUSED,
                .din  = I2S_GPIO_UNUSED,
            },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_clk_chan, &std_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_clk_chan));
    }
    ESP_LOGI(TAG, "APLL MCLK running on GPIO %d at %d Hz",
             PIN_MCLK_OUT, SAMPLE_RATE * MCLK_MULT);

    /* Give MCLK a moment to stabilize */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* --- Step 2: Set up PARLIO TX manually (ext clk on GPIO 21) --- */
    ESP_LOGI(TAG, "Setting up PARLIO TX (ext clk on GPIO %d)...", PIN_PARLIO_CLK);
    parlio_tx_unit_handle_t parlio_unit = NULL;
    {
        uint32_t mclk_freq = MCLK_MULT * SAMPLE_RATE;
        uint32_t bclk_freq = SLOT_WIDTH * 2 * SAMPLE_RATE; /* stereo = 2 slots */

        parlio_tx_unit_config_t pcfg = {
            .clk_src = PARLIO_CLK_SRC_DEFAULT,
            .clk_in_gpio_num = PIN_PARLIO_CLK,
            .input_clk_src_freq_hz = mclk_freq,
            .output_clk_freq_hz = bclk_freq,
            .data_width = 2,                     /* TXD[0]=LRCK, TXD[1]=DATA */
            .clk_out_gpio_num = PIN_BCLK,
            .valid_gpio_num = -1,
            .trans_queue_depth = 8,
            .max_transfer_size = TEST_FRAMES * 2 * 2, /* frames * bclk_per_frame * bytes */
            .dma_burst_size = 0,
            .sample_edge = PARLIO_SAMPLE_EDGE_POS,
            .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
            .flags = { .clk_gate_en = 0, .io_loop_back = 0 },
        };
        for (int i = 0; i < 16; i++) pcfg.data_gpio_nums[i] = -1;
        pcfg.data_gpio_nums[0] = PIN_LRCK;
        pcfg.data_gpio_nums[1] = PIN_DATA;

        ESP_ERROR_CHECK(parlio_new_tx_unit(&pcfg, &parlio_unit));
        ESP_ERROR_CHECK(parlio_tx_unit_enable(parlio_unit));
    }
    ESP_LOGI(TAG, "PARLIO TX enabled: BCLK=%d, LRCK=%d, DATA=%d",
             PIN_BCLK, PIN_LRCK, PIN_DATA);

    /* --- Step 3: Set up I2S RX (slave, reads from PARLIO outputs) --- */
    ESP_LOGI(TAG, "Setting up I2S RX (slave mode)...");
    i2s_chan_handle_t rx_chan = NULL;
    ESP_ERROR_CHECK(setup_i2s_rx(&rx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_LOGI(TAG, "I2S RX enabled in slave mode");

    /* --- Step 4: Pre-fill PARLIO with silence to get clocks running --- */
    ESP_LOGI(TAG, "Sending silence to establish clock...");
    {
        /* Encode silence frames: LRCK toggles, data = 0.
         * data_width=2: each byte holds 4 BCLK cycles (2 bits each, LSB packed) */
        uint16_t bclk_per_frame = SLOT_WIDTH * 2;
        size_t frame_bytes = bclk_per_frame; /* 2-bit width: 4 cycles per byte -> 64/4=16, but actually data_width=2 means each parallel word is 2 bits, packed LSB. With bit_pack_order LSB: 4 words per byte. So 64 bclk = 16 bytes */

        /* Actually with data_width=2, PARLIO packs 4 x 2-bit words per byte (LSB order).
         * So bclk_per_frame / 4 = bytes per frame. For 64 BCLK -> 16 bytes. */
        frame_bytes = bclk_per_frame / 4;
        if (frame_bytes * 4 < bclk_per_frame) frame_bytes++;

        size_t buf_size = frame_bytes * 4; /* a few frames of silence */
        uint8_t *silence_buf = heap_caps_calloc(1, buf_size,
                                                 MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!silence_buf) {
            ESP_LOGE(TAG, "alloc silence buf failed");
            return;
        }

        /* Encode LRCK toggling with zero data for several frames */
        size_t idx = 0;
        for (int frame = 0; frame < 4; frame++) {
            for (uint16_t bclk = 0; bclk < bclk_per_frame; bclk++) {
                uint8_t slot = bclk / SLOT_WIDTH;
                uint8_t lrck = (slot >= 1) ? 1 : 0; /* Philips */
                uint8_t data_bit = 0;
                uint8_t word = (lrck & 1) | ((data_bit & 1) << 1); /* 2-bit: [0]=LRCK, [1]=DATA */

                /* Pack into byte: 4 x 2-bit words per byte, LSB first */
                size_t byte_pos = idx / 4;
                size_t bit_pos  = (idx % 4) * 2;
                if (byte_pos < buf_size) {
                    silence_buf[byte_pos] |= (word & 0x03) << bit_pos;
                }
                idx++;
            }
        }

        parlio_transmit_config_t tcfg = { .idle_value = 0 };
        ESP_ERROR_CHECK(parlio_tx_unit_transmit(parlio_unit, silence_buf,
                                                 idx * 2, &tcfg)); /* bits = idx * data_width */
        ESP_ERROR_CHECK(parlio_tx_unit_wait_all_done(parlio_unit, 1000));
        free(silence_buf);
    }

    /* Drain any stale RX data */
    {
        size_t rx_bytes = 0;
        int32_t drain_buf[TEST_FRAMES * 2];
        i2s_channel_read(rx_chan, drain_buf, sizeof(drain_buf), &rx_bytes, 200);
        ESP_LOGI(TAG, "drained %u bytes of stale RX data", (unsigned)rx_bytes);
    }

    /* --- Step 5: Test loop --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Starting loopback tests ===");

    int32_t *tx_buf = malloc(TEST_FRAMES * 2 * sizeof(int32_t));
    int32_t *rx_buf = calloc(TEST_FRAMES * 4, sizeof(int32_t)); /* extra space for alignment search */
    uint8_t *parlio_buf = heap_caps_calloc(1, TEST_FRAMES * SLOT_WIDTH * 2 / 4 + 16,
                                            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!tx_buf || !rx_buf || !parlio_buf) {
        ESP_LOGE(TAG, "alloc test buffers failed");
        return;
    }

    uint32_t total_tested = 0;
    uint32_t total_matched = 0;
    const uint16_t bclk_per_frame = SLOT_WIDTH * 2;

    for (int iteration = 0; iteration < 10; iteration++) {
        /* Generate test pattern */
        uint32_t start_idx = iteration * TEST_FRAMES;
        generate_test_pattern(tx_buf, TEST_FRAMES, start_idx);

        /* Encode TX buffer into PARLIO format */
        memset(parlio_buf, 0, TEST_FRAMES * bclk_per_frame / 4 + 16);
        size_t pidx = 0;
        size_t total_parlio_bytes = 0;
        for (size_t f = 0; f < TEST_FRAMES; f++) {
            int32_t left  = tx_buf[f * 2 + 0];
            int32_t right = tx_buf[f * 2 + 1];

            for (uint16_t bclk = 0; bclk < bclk_per_frame; bclk++) {
                uint8_t slot = bclk / SLOT_WIDTH;
                uint16_t pos = bclk % SLOT_WIDTH;
                int16_t bit_idx = (int16_t)pos - 1; /* Philips 1-BCLK offset */

                uint8_t lrck = (slot >= 1) ? 1 : 0;
                uint8_t data_bit = 0;
                if (bit_idx >= 0 && bit_idx < BITS) {
                    int32_t samp = (slot == 0) ? left : right;
                    data_bit = (samp >> (31 - bit_idx)) & 1;
                }

                uint8_t word = (lrck & 1) | ((data_bit & 1) << 1);
                size_t byte_pos = pidx / 4;
                size_t bit_pos  = (pidx % 4) * 2;
                parlio_buf[byte_pos] |= (word & 0x03) << bit_pos;
                pidx++;
            }
        }
        total_parlio_bytes = (pidx + 3) / 4;

        /* Transmit via PARLIO */
        parlio_transmit_config_t tcfg = { .idle_value = 0 };
        ESP_ERROR_CHECK(parlio_tx_unit_transmit(parlio_unit, parlio_buf,
                                                 pidx * 2, &tcfg));

        /* Read from I2S RX */
        size_t rx_bytes = 0;
        size_t rx_total = 0;
        size_t rx_buf_size = TEST_FRAMES * 2 * sizeof(int32_t);

        /* Read in chunks, accumulating */
        while (rx_total < rx_buf_size) {
            esp_err_t ret = i2s_channel_read(rx_chan,
                                              (uint8_t *)rx_buf + rx_total,
                                              rx_buf_size - rx_total,
                                              &rx_bytes, 500);
            if (ret == ESP_ERR_TIMEOUT || rx_bytes == 0) break;
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(ret));
                break;
            }
            rx_total += rx_bytes;
        }

        size_t rx_frames = rx_total / (2 * sizeof(int32_t));

        /* Wait for PARLIO to finish */
        parlio_tx_unit_wait_all_done(parlio_unit, 1000);

        if (rx_frames == 0) {
            ESP_LOGW(TAG, "iteration %d: no RX data received", iteration);
            continue;
        }

        /* Compare */
        int matched = compare_buffers(tx_buf, TEST_FRAMES, rx_buf, rx_frames);
        total_tested += TEST_FRAMES;
        total_matched += matched;

        ESP_LOGI(TAG, "iter %d: TX %zu frames, RX %zu frames, matched %d",
                 iteration, (size_t)TEST_FRAMES, rx_frames, matched);

        /* Print first few samples for visual inspection */
        if (iteration == 0) {
            ESP_LOGI(TAG, "  TX[0]: L=0x%08lx R=0x%08lx",
                     (unsigned long)tx_buf[0], (unsigned long)tx_buf[1]);
            ESP_LOGI(TAG, "  TX[1]: L=0x%08lx R=0x%08lx",
                     (unsigned long)tx_buf[2], (unsigned long)tx_buf[3]);
            if (rx_frames >= 2) {
                ESP_LOGI(TAG, "  RX[0]: L=0x%08lx R=0x%08lx",
                         (unsigned long)rx_buf[0], (unsigned long)rx_buf[1]);
                ESP_LOGI(TAG, "  RX[1]: L=0x%08lx R=0x%08lx",
                         (unsigned long)rx_buf[2], (unsigned long)rx_buf[3]);
            }
        }
    }

    /* --- Results --- */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== RESULTS ===");
    ESP_LOGI(TAG, "Total frames tested: %"PRIu32, total_tested);
    ESP_LOGI(TAG, "Total frames matched: %"PRIu32, total_matched);
    if (total_tested > 0) {
        float pct = 100.0f * (float)total_matched / (float)total_tested;
        ESP_LOGI(TAG, "Match rate: %.2f%%", pct);
        if (pct > 99.0f) {
            ESP_LOGI(TAG, "PASS -- loopback verified");
        } else if (pct > 50.0f) {
            ESP_LOGW(TAG, "PARTIAL -- some data corruption, check signal integrity");
        } else if (total_matched > 0) {
            ESP_LOGW(TAG, "LOW MATCH -- likely alignment or timing issue");
        } else {
            ESP_LOGE(TAG, "FAIL -- no matching data, check wiring and clock");
        }
    } else {
        ESP_LOGE(TAG, "FAIL -- no data received at all");
    }

    /* Cleanup */
    free(tx_buf);
    free(rx_buf);
    free(parlio_buf);
    i2s_channel_disable(rx_chan);
    i2s_del_channel(rx_chan);
    parlio_tx_unit_disable(parlio_unit);
    parlio_del_tx_unit(parlio_unit);
    i2s_channel_disable(i2s_clk_chan);
    i2s_del_channel(i2s_clk_chan);

    ESP_LOGI(TAG, "test complete, entering idle");
    while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
