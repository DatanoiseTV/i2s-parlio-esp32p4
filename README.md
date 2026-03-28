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
    |                   +-------+          |         |         |
    |                                      | wire    |         |
    |   +----------------------------------|---------+         |
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

This makes the encoder ~4x faster than bit-by-bit extraction and enables real-time output at all TDM configurations up to TDM8 x 2 lines (16 channels) at 48 kHz.

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

Hardware-verified on ESP32-P4 at 360 MHz using PCNT (pulse counter) to measure actual BCLK edges on the output pin:

| Configuration | Channels | BCLK Rate | Measured Fs | Status |
|---------------|----------|-----------|-------------|--------|
| I2S Stereo x1 | 2 | 3.072 MHz | 48000 Hz | PASS |
| I2S Stereo x2 | 4 | 3.072 MHz | 48000 Hz | PASS |
| TDM4 x1 | 4 | 6.144 MHz | 48000 Hz | PASS |
| TDM8 x1 | 8 | 12.288 MHz | 48000 Hz | PASS |
| TDM4 x2 | 8 | 6.144 MHz | 48000 Hz | PASS |
| TDM8 x2 | 16 | 12.288 MHz | 48000 Hz | PASS |

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

### Quick Example

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
