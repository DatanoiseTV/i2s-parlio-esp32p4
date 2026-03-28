# PARLIO Audio Transmitter for ESP32-P4

Multi-protocol audio transmitter that creatively repurposes the Parallel IO peripheral on the ESP32-P4 to synthesize digital audio output signals, clocked by the APLL for sample-accurate timing.

```
                          ESP32-P4
    +----------------------------------------------------------+
    |                                                          |
    |   +-------+      +-------+      +---------+              |
    |   | APLL  |----->|  I2S  |----->| MCLK    |-----> GPIO   |
    |   | (PLL) |      | (clk  |      | output  |   |          |
    |   +-------+      |  only)|      +---------+   |          |
    |                  +-------+           |        |          |
    |                                      | wire   |          |
    |   +----------------------------------|--------+          |
    |   |                                  v                   |
    |   |  +---------+    +--------+   +---------+             |
    |   |  | PARLIO  |<---| MCLK   |<--| ext clk |             |
    |   |  |  TX     |    | divider|   | input   |             |
    |   |  |         |    +--------+   +---------+             |
    |   |  |         |         |                               |
    |   |  |         |    +--------+                           |
    |   |  |         |--->| clk_out|---------> BCLK            |
    |   |  |         |    +--------+                           |
    |   |  |         |                                         |
    |   |  |  DMA    |    +--------+                           |
    |   |  |  buf[]--|--->| TXD[0] |---------> LRCK / FSYNC    |
    |   |  |         |    +--------+                           |
    |   |  |         |    +--------+                           |
    |   |  |         |--->| TXD[1] |---------> DATA 0          |
    |   |  |         |    +--------+                           |
    |   |  |         |    +--------+                           |
    |   |  |         |--->| TXD[2] |---------> DATA 1          |
    |   |  |         |    +--------+                           |
    |   |  |         |       ...                               |
    |   |  |         |    +--------+                           |
    |   |  |         |--->|TXD[15] |---------> DATA 14         |
    |   |  +---------+    +--------+                           |
    |   |                                                      |
    +----------------------------------------------------------+
```

## The Trick: Three Peripherals, One Clock

We chain three peripherals together in ways they were never designed for:

```
  APLL (audio PLL)     I2S peripheral        GPIO matrix         PARLIO peripheral
  +--------------+     +-------------+     +-------------+     +-----------------+
  | Generates    |---->| Used ONLY   |---->| Same GPIO   |---->| Receives MCLK   |
  | precise audio|     | for MCLK    |     | is both     |     | as ext clock    |
  | clock        |     | generation  |     | output and  |     | input, divides  |
  | (12.288 MHz) |     | (no audio!) |     | input       |     | to BCLK, shifts |
  +--------------+     +-------------+     +-------------+     | out parallel    |
                                                               | DMA data on     |
                                                               | every BCLK tick |
                                                               +-----------------+
```

**Peripheral 1 -- APLL**: Generates a precise audio master clock. Standard audio frequencies (12.288 MHz for 48 kHz, 11.2896 MHz for 44.1 kHz) are native to this PLL.

**Peripheral 2 -- I2S**: Misused purely as a clock conduit. We allocate an I2S TX channel, configure it with `I2S_CLK_SRC_APLL`, but never write audio data to it. Its only job is to output MCLK on a GPIO pin. The I2S BCLK/WS/DOUT pins are left disconnected (or optionally used for real audio output as a "free" bonus).

**Peripheral 3 -- PARLIO**: Designed for parallel display interfaces (LCD, LED matrices). We repurpose it as a multi-channel I2S transmitter. It reads the MCLK from the GPIO (via the GPIO matrix), divides it down to BCLK, and shifts out pre-computed bit patterns from DMA buffers on every BCLK cycle.

### Why PARLIO and not just I2S?

The ESP32-P4 has 3 I2S peripherals, each with at most 2 data output pins. PARLIO has **16 parallel data lines** that all shift simultaneously. By encoding LRCK on TXD[0] and audio data on TXD[1..15], we get up to **15 independent data outputs** from a single peripheral -- far more than I2S can provide.

## How the DMA Buffer Encodes I2S Signals

PARLIO shifts out one byte per BCLK tick across all TXD lines simultaneously:

```
  DMA buffer (one byte per BCLK cycle):

  Byte layout:  [bit 0]  [bit 1]  [bit 2]  [bit 3]  ...  [bit 7]
                  LRCK    DATA 0   DATA 1   DATA 2         unused

  Example: stereo 32-bit, 1 data line (64 bytes per frame):

  BCLK  0: LRCK=0, D=0      --> 0x00  (left slot, padding)
  BCLK  1: LRCK=0, D=MSB(L) --> 0x00 or 0x02
  BCLK  2: LRCK=0, D=bit30  --> 0x00 or 0x02
    ...
  BCLK 31: LRCK=0, D=bit1   --> 0x00 or 0x02
  BCLK 32: LRCK=1, D=0      --> 0x01  (right slot, padding)
  BCLK 33: LRCK=1, D=MSB(R) --> 0x01 or 0x03
    ...
  BCLK 63: LRCK=1, D=bit1   --> 0x01 or 0x03
```

### The LUT Encoder

Instead of extracting one bit at a time (31 shift+mask+store per channel), we use a precomputed lookup table:

```
  For each data line at TXD position P, precompute:
    line_lut[P][byte_value] = 8 output bytes as uint64_t

  Input: one byte of sample (8 bits)
  Output: 8 DMA bytes with each bit placed at position P

  Example for line 0 (position 1), input byte 0xA5 = 10100101:
    line_lut[0][0xA5] = { 0x02, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x02 }

  A 32-bit sample = 4 bytes = 4 LUT lookups = 4 uint64_t writes
  Multiple data lines: OR the LUT results together
```

This makes the encoder ~4x faster than bit-by-bit extraction and enables real-time output at all tested configurations up to TDM16 x 11 lines (176 channels) at 48 kHz, including simultaneous ADAT + I2S multi-protocol output.

### Gapless DMA: Loop Transmission with Ping-Pong Buffers

```
  Buffer rotation (3 buffers):

  Time -->
  DMA:     [===buf0===][===buf1===][===buf2===][===buf0===]...
  Encoder:        [fill1]   [fill2]   [fill0]   [fill1]...

  buf0 plays (loop) while encoder fills buf1
  Submit buf1 --> DMA chains seamlessly via gdma_link_concat
  buf1 plays (loop) while encoder fills buf2
  Submit buf2 --> seamless chain, buf0 now free
  ...

  Zero gap between buffers (DMA descriptor chaining in hardware)
```

The PARLIO driver's `loop_transmission` mode keeps the DMA running continuously. When a new buffer is submitted, the DMA hardware chains to it without stopping. The encoder paces submissions using `esp_timer_get_time()` with frame-count-based absolute timing to prevent drift.

## Supported Protocols

### I2S / TDM (via PARLIO)

| Mode | Slots/Line | LRCK Behavior | Max Channels (15 lines) |
|------|-----------|---------------|-------------------------|
| Standard I2S | 2 | 50/50 duty cycle | **30** |
| TDM4 | 4 | 1-BCLK frame sync | **60** |
| TDM8 | 8 | 1-BCLK frame sync | **120** |
| TDM16 | 16 | 1-BCLK frame sync | **240** |

### S/PDIF (via PARLIO, 1-bit serial)

Biphase mark coded stereo output at 128 * Fs on a single GPIO. Consumer or professional channel status.

### ADAT Lightpipe (via PARLIO, 1-bit serial)

NRZI encoded 8-channel 24-bit output at 256 * Fs on a single GPIO.

### I2S HW Passthrough

The I2S peripheral used for clock generation can also output real audio on its own pins. I2S0 has 2 data outputs (up to 32 TDM16 channels), I2S1/I2S2 have 1 each (16 channels).

### Maximum Channel Counts

| Scenario | Breakdown | Total |
|----------|-----------|-------|
| PARLIO TDM16 only | 15 lines x 16 slots | **240 ch** |
| PARLIO TDM16 + all I2S HW | 240 + 32 + 16 + 16 | **304 ch** |
| Mixed: PARLIO I2S + SPDIF + ADAT + I2S0 | 24 + 2 + 8 + 32 | **66 ch** |

All outputs share the same APLL clock -- zero inter-channel and inter-protocol clock drift.

## Performance

Hardware-verified on ESP32-P4 at 360 MHz using PCNT (pulse counter) to count actual clock edges on the output pin. **40 tests, 40 PASS**, sub-ppm accuracy:

| Configuration | Channels | Clock Rate | Measured Fs | Error |
|---------------|----------|------------|-------------|-------|
| I2S Stereo x1 | 2 | 3.072 MHz | 48000.0 Hz | -1 ppm |
| I2S Stereo x11 | 22 | 3.072 MHz | 48000.0 Hz | -1 ppm |
| TDM4 x11 | 44 | 6.144 MHz | 48000.0 Hz | 0 ppm |
| TDM8 x11 | 88 | 12.288 MHz | 48000.0 Hz | 0 ppm |
| TDM16 x11 | 176 | 24.576 MHz | 48000.0 Hz | -1 ppm |
| ADAT only | 8 | 12.288 MHz | 48000.0 Hz | +1 ppm |
| ADAT + Stereo x4 | 16 | 12.288 MHz | 48000.0 Hz | -1 ppm |
| ADAT + TDM8 x4 | 40 | 12.288 MHz | 48000.0 Hz | 0 ppm |
| ADAT + TDM4 x7 | 36 | 12.288 MHz | 48000.0 Hz | -1 ppm |
| ADAT + Stereo x8 | 24 | 12.288 MHz | 48000.0 Hz | 0 ppm |
| Stereo 96 kHz | 2 | 6.144 MHz | 96000.0 Hz | 0 ppm |
| TDM8 x4 96 kHz | 32 | 24.576 MHz | 96000.0 Hz | -1 ppm |
| Stereo 192 kHz | 2 | 12.288 MHz | 192000.0 Hz | -1 ppm |
| Stereo x11 192 kHz | 22 | 12.288 MHz | 192000.0 Hz | 0 ppm |
| Stereo 44.1 kHz | 2 | 2.822 MHz | 44100.0 Hz | -1 ppm |
| TDM8 x4 44.1 kHz | 32 | 11.290 MHz | 44100.0 Hz | -1 ppm |
| Stereo 8 kHz | 2 | 0.512 MHz | 8000.0 Hz | 0 ppm |

Full test suite: 40 tests across 8 kHz to 192 kHz, stereo to TDM16, standalone I2S and unified ADAT+I2S combos, 1 to 11 data lines. All PASS within 2 ppm.

### Key Techniques

- **LUT encoder**: per-line lookup table, 4 uint64_t writes per sample per line
- **Loop DMA**: `parlio_tx_unit_transmit(loop_transmission=true)` + `gdma_link_concat` = zero-gap buffer chaining
- **Absolute frame-count timing**: `total_submitted * 1000000 / sample_rate` prevents truncation drift
- **Dedicated CPU1 task**: encoder at highest priority, no system task competition
- **PCNT verification**: hardware pulse counter on BCLK pin confirms exact output frequency

## Latency

Output latency = `frames_per_buffer * 3 / sample_rate` (3 rotating DMA buffers).

| frames_per_buffer | Latency @ 48 kHz |
|-------------------|-------------------|
| 128 | 8 ms |
| 64 | 4 ms |
| 32 | 2 ms |

All signals (BCLK, LRCK, data lines) are in the same PARLIO clock domain, phase-aligned to within a single MCLK period (< 100 ns).

## Slot Width and BCLK Rate

The `slot_width` parameter controls BCLK cycles per slot. Some codecs expect more BCLK than the sample width:

| Codec Expects | bits_per_sample | slot_width | BCLK/frame (stereo) |
|--------------|----------------|-----------|---------------------|
| 64fs (typical) | 16/24/32 | 32 | 64 |
| 128fs | 16/24/32 | 64 | 128 |
| 256fs | 16/24/32 | 128 | 256 |

### MCLK Requirements

`mclk_multiple` must be >= `slot_width * num_slots` and divide evenly.

| Mode | slot_width=32 | Min `mclk_multiple` | MCLK @ 48 kHz |
|------|--------------|---------------------|---------------|
| Standard | 64 BCLK/frame | 128 | 6.144 MHz |
| TDM4 | 128 BCLK/frame | 256 | 12.288 MHz |
| TDM8 | 256 BCLK/frame | 512 | 24.576 MHz |
| TDM16 | 512 BCLK/frame | 1024 | 49.152 MHz |

## Build

```bash
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

Requires ESP-IDF v5.4+ with ESP32-P4 support.

## Component API

### Standalone drivers

- `parlio_i2s.h` -- I2S/TDM (BCLK on clk_out, LUT encoder, loop DMA)
- `parlio_spdif_tx.h` -- S/PDIF (1-bit BMC serial)
- `parlio_adat_tx.h` -- ADAT (1-bit NRZI serial)

### Unified multi-protocol driver

- `parlio_audio_tx.h` -- any combination of I2S + S/PDIF + ADAT + I2S HW passthrough

### Example 1: Standalone TDM8 (16 channels)

```c
parlio_i2s_tx_config_t cfg = {
    .sample_rate    = 48000,
    .bits_per_sample = 32,
    .slot_width     = 32,
    .num_data_lines = 2,
    .mclk_multiple  = 512,
    .mode           = PARLIO_I2S_MODE_TDM8,
    .mclk_gpio      = GPIO_NUM_21,
    .bclk_gpio      = GPIO_NUM_22,
    .lrck_gpio      = GPIO_NUM_23,
    .data_gpios     = { GPIO_NUM_24, GPIO_NUM_25 },
    .frames_per_buffer = 128,
};

parlio_i2s_tx_handle_t tx;
parlio_i2s_tx_new(&cfg, &tx);
parlio_i2s_tx_enable(tx);

/* 16 channels: [line0_slot0..slot7, line1_slot0..slot7] */
int32_t samples[16];
size_t written;
parlio_i2s_tx_write(tx, samples, 1, &written, 1000);
```

### Example 2: ADAT + Stereo I2S (10 channels, unified driver)

8 channels via ADAT optical + 2 channels via I2S, all sample-locked:

```c
parlio_audio_i2s_config_t i2s_cfg = {
    .mode = PARLIO_AUDIO_I2S_STANDARD,
    .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 1,
    .bclk_gpio = GPIO_NUM_24, .lrck_gpio = GPIO_NUM_25,
    .data_gpios = { GPIO_NUM_32 },
};
parlio_audio_adat_config_t adat_cfg = { .adat_gpio = GPIO_NUM_33 };

parlio_audio_tx_config_t cfg = {
    .sample_rate = 48000, .mclk_multiple = 512,
    .mclk_gpio = GPIO_NUM_21, .clk_out_gpio = GPIO_NUM_22,
    .i2s = &i2s_cfg, .adat = &adat_cfg,
};

parlio_audio_tx_handle_t tx;
parlio_audio_tx_new(&cfg, &tx);
parlio_audio_tx_enable(tx);

/* 10 samples per frame: [I2S_L, I2S_R, ADAT_ch0..ch7] */
int32_t frame[10];
parlio_audio_tx_write(tx, frame, 1, NULL, 1000);
```

### Example 3: S/PDIF + ADAT + I2S (14 channels, all three protocols)

2 channels S/PDIF + 8 channels ADAT + 4 channels I2S stereo, simultaneously:

```c
parlio_audio_i2s_config_t i2s_cfg = {
    .mode = PARLIO_AUDIO_I2S_STANDARD,
    .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 2,
    .bclk_gpio = GPIO_NUM_24, .lrck_gpio = GPIO_NUM_25,
    .data_gpios = { GPIO_NUM_32, GPIO_NUM_33 },
};
parlio_audio_spdif_config_t spdif_cfg = {
    .bits_per_sample = 24, .consumer_format = true,
    .spdif_gpio = GPIO_NUM_36,
};
parlio_audio_adat_config_t adat_cfg = { .adat_gpio = GPIO_NUM_45 };

parlio_audio_tx_config_t cfg = {
    .sample_rate = 48000, .mclk_multiple = 512,
    .mclk_gpio = GPIO_NUM_21, .clk_out_gpio = GPIO_NUM_22,
    .i2s = &i2s_cfg, .spdif = &spdif_cfg, .adat = &adat_cfg,
};

parlio_audio_tx_handle_t tx;
parlio_audio_tx_new(&cfg, &tx);
parlio_audio_tx_enable(tx);

/* 14 samples per frame: [I2S_L0, I2S_R0, I2S_L1, I2S_R1, SPDIF_L, SPDIF_R, ADAT_0..7] */
size_t frame_size = parlio_audio_tx_get_frame_size(tx); /* = 14 */
int32_t frame[14];
parlio_audio_tx_write(tx, frame, 1, NULL, 1000);
```

### Example 4: S/PDIF only (2 channels, standalone driver)

```c
parlio_spdif_tx_config_t cfg = {
    .sample_rate = 48000,
    .bits_per_sample = 24,
    .mclk_multiple = 256,
    .mclk_gpio = GPIO_NUM_21,
    .spdif_gpio = GPIO_NUM_22,
    .consumer_format = true,
};

parlio_spdif_tx_handle_t spdif;
parlio_spdif_tx_new(&cfg, &spdif);
parlio_spdif_tx_enable(spdif);

int32_t stereo[2] = { left, right };
parlio_spdif_tx_write(spdif, stereo, 1, NULL, 1000);
```

### Example 5: ADAT only (8 channels, standalone driver)

```c
parlio_adat_tx_config_t cfg = {
    .sample_rate = 48000,
    .mclk_multiple = 512,
    .mclk_gpio = GPIO_NUM_21,
    .adat_gpio = GPIO_NUM_22,
};

parlio_adat_tx_handle_t adat;
parlio_adat_tx_new(&cfg, &adat);
parlio_adat_tx_enable(adat);

int32_t frame[8] = { ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7 };
parlio_adat_tx_write(adat, frame, 1, NULL, 1000);
```

### Example 6: I2S HW passthrough (bonus output from clock generator)

The I2S peripheral used for APLL clock generation can also output real audio:

```c
parlio_audio_i2s_hw_config_t hw_cfg = {
    .bits_per_sample = 32,
    .total_slots = 8,                   /* TDM8 on the I2S HW peripheral */
    .bclk_gpio = GPIO_NUM_46,
    .ws_gpio   = GPIO_NUM_47,
    .dout_gpio = GPIO_NUM_48,
    .dout2_gpio = -1,
};

parlio_audio_tx_config_t cfg = {
    .sample_rate = 48000, .mclk_multiple = 512,
    .mclk_gpio = GPIO_NUM_21, .clk_out_gpio = GPIO_NUM_22,
    .adat = &adat_cfg,      /* 8 ch ADAT via PARLIO */
    .i2s_hw = &hw_cfg,      /* 8 ch TDM via I2S HW (free, same APLL) */
};

parlio_audio_tx_handle_t tx;
parlio_audio_tx_new(&cfg, &tx);
parlio_audio_tx_enable(tx);

/* ADAT: write via PARLIO path */
parlio_audio_tx_write(tx, adat_samples, num_frames, NULL, 1000);

/* I2S HW: write via standard ESP-IDF API (sample-locked with PARLIO) */
i2s_chan_handle_t hw = parlio_audio_tx_get_i2s_hw_handle(tx);
i2s_channel_write(hw, tdm_samples, size, &bytes_written, 1000);
```

## Benchmarks

```
Configuration                   Fs   Ch      HW Fs    Error   SW Fs  Status
------------------------------------------------------------------------
Stereo x1                   48000    2    47999.9     -1ppm    47982  PASS
Stereo x2                   48000    4    48000.0     -0ppm    48055  PASS
Stereo x4                   48000    8    47999.9     -1ppm    48058  PASS
Stereo x7                   48000   14    48000.0     -0ppm    48061  PASS
TDM4 x1 (4ch)               48000    4    48000.0     -0ppm    48052  PASS
TDM4 x2 (8ch)               48000    8    48000.0     -0ppm    48052  PASS
TDM4 x4 (16ch)              48000   16    48000.0     -0ppm    48054  PASS
TDM4 x7 (28ch)              48000   28    48000.0     -0ppm    48054  PASS
TDM8 x1 (8ch)               48000    8    48000.0     -0ppm    48053  PASS
TDM8 x2 (16ch)              48000   16    48000.0     -0ppm    48056  PASS
TDM8 x4 (32ch)              48000   32    48000.0     -0ppm    48056  PASS
TDM8 x7 (56ch)              48000   56    48000.0     -0ppm    48056  PASS
Stereo x1                   96000    2    96000.0     -0ppm    96121  PASS
Stereo x4                   96000    8    96000.0     -0ppm    96116  PASS
TDM4 x1 (4ch)               96000    4    96000.1     +1ppm    96107  PASS
TDM4 x4 (16ch)              96000   16    96000.0     -0ppm    96112  PASS
TDM8 x1 (8ch)               96000    8    96000.0     -0ppm    96107  PASS
TDM8 x2 (16ch)              96000   16    96000.0     -0ppm    96112  PASS
Stereo x1                  192000    2   191999.9     -0ppm   192229  PASS
Stereo x4                  192000    8   191999.9     -0ppm   192243  PASS
TDM4 x1 (4ch)              192000    4   191999.9     -0ppm   192231  PASS
Stereo 44.1k                44100    2    44100.0     -0ppm    44140  PASS
TDM8 x2 44.1k               44100   16    44100.0     -1ppm    44142  PASS
Stereo 88.2k                88200    2    88199.9     -1ppm    88296  PASS
Stereo 176.4k              176400    2   176399.9     -1ppm   176626  PASS
Stereo 8kHz                  8000    2     8000.0     -0ppm     7987  PASS
Stereo 16kHz                16000    2    16000.0     -0ppm    16003  PASS
Stereo 22.05kHz             22050    2    22050.0     -0ppm    22071  PASS
Stereo 32kHz                32000    2    32000.0     -0ppm    32031  PASS
------------------------------------------------------------------------
Total: 30 tests | 29 PASS 1 SKIP
```

