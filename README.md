# PARLIO Audio Transmitter for ESP32-P4

Multi-protocol audio transmitter that abuses the Parallel IO peripheral on the ESP32-P4 to synthesize digital audio output signals, clocked by the APLL for sample-accurate timing. All outputs can run simultaneously from a single PARLIO TX unit, sample-locked with zero inter-protocol skew.

Supported protocols (any combination, all simultaneous):
- **I2S / TDM** (via PARLIO) -- up to 240 channels (15 data lines x 16 TDM slots)
- **S/PDIF** -- stereo IEC60958 with biphase mark coding, single wire
- **ADAT Lightpipe** -- 8 channels of 24-bit audio with NRZI encoding, single wire
- **I2S / TDM** (via I2S HW) -- the I2S peripheral used for clock generation can simultaneously output real audio on its own pins (up to 32 extra channels via TDM16 on I2S0)

### Maximum channel counts

| Scenario | Breakdown | Total |
|----------|-----------|-------|
| PARLIO TDM16 only | 15 lines x 16 slots | **240 ch** |
| PARLIO TDM16 + I2S HW TDM16 (all 3 peripherals) | 240 + 32 + 16 + 16 | **304 ch** |
| PARLIO I2S (12 lines) + SPDIF + ADAT + I2S0 TDM16 x2 | 24 + 2 + 8 + 32 | **66 ch** (mixed protocol) |
| PARLIO ADAT + I2S0 TDM16 x2 + I2S1 TDM16 + I2S2 TDM16 | 8 + 32 + 16 + 16 | **72 ch** (with ADAT) |

All outputs share the same APLL clock -- zero inter-channel and inter-protocol clock drift

## How it works

The ESP32-P4's PARLIO peripheral is a parallel data output engine with DMA support, designed for display interfaces. We repurpose it as a multi-channel I2S/TDM transmitter:

1. **APLL** generates a precise audio master clock via the I2S peripheral
2. The **I2S MCLK pin** outputs MCLK directly to the DAC and simultaneously feeds into **PARLIO as external clock input** (same GPIO, via GPIO matrix)
3. **PARLIO** divides MCLK down to BCLK internally and outputs it on the **dedicated `clk_out` pin** -- this does not consume a data line
4. PARLIO parallel data bus carries:
   - **TXD[0]** = LRCK / frame sync
   - **TXD[1..N]** = audio data lines (up to 15)

Each BCLK tick, PARLIO shifts out one parallel word containing the LRCK state and all data line bits. The DMA buffer contains pre-computed bit patterns -- one word per BCLK cycle.

### Why BCLK on clk_out matters

By using the dedicated clock output pin for BCLK instead of a data line:
- **15 data lines** available instead of 14 (one extra GPIO freed)
- **DMA bandwidth reduced** by the MCLK/BCLK ratio (typically 4x less traffic)
- Only one GPIO needed for MCLK (serves as both DAC output and PARLIO clock input)
- No separate loopback wire needed

## Supported Modes

| Mode | Slots/Line | LRCK Behavior | Typical Use |
|------|-----------|---------------|-------------|
| `PARLIO_I2S_MODE_STANDARD` | 2 | 50/50 duty cycle (Philips I2S) | Stereo DACs, codecs |
| `PARLIO_I2S_MODE_TDM4` | 4 | 1-BCLK frame sync pulse | 4-ch DACs, small arrays |
| `PARLIO_I2S_MODE_TDM8` | 8 | 1-BCLK frame sync pulse | Multi-ch audio interfaces |
| `PARLIO_I2S_MODE_TDM16` | 16 | 1-BCLK frame sync pulse | High-density systems |

## Channel Counts

Each PARLIO data line carries `num_slots` audio channels. With up to 15 data lines:

| Mode | Channels/Line | Max Lines | Max Channels |
|------|--------------|-----------|-------------|
| Standard I2S | 2 | 15 | **30** |
| TDM4 | 4 | 15 | **60** |
| TDM8 | 8 | 15 | **120** |
| TDM16 | 16 | 15 | **240** |

## Slot Width and BCLK Rate

The `slot_width` parameter controls how many BCLK cycles each slot occupies. The total BCLK cycles per frame is `slot_width * num_slots`. Audio data is MSB-first within each slot, with unused bit positions zero-padded.

Some codecs expect more BCLK cycles per frame than the audio data strictly requires. For example, a stereo 24-bit codec running at 256fs BCLK would use `slot_width = 128` (128 bits/slot x 2 slots = 256 BCLK/frame). Set `slot_width` to match whatever your codec expects:

| Codec Expects | bits_per_sample | slot_width | BCLK/frame (stereo) |
|--------------|----------------|-----------|---------------------|
| 32fs | 16 | 16 | 32 |
| 48fs | 24 | 24 | 48 |
| 64fs (typical) | 16/24/32 | 32 | 64 |
| 128fs | 16/24/32 | 64 | 128 |
| 256fs | 16/24/32 | 128 | 256 |

### MCLK Requirements

The MCLK multiple (`mclk_multiple`) must satisfy:
- `mclk_multiple >= slot_width * num_slots` (MCLK/BCLK ratio must be at least 1)
- `mclk_multiple % (slot_width * num_slots) == 0` (must divide evenly)

| Mode | slot_width=32 | Min `mclk_multiple` | MCLK @ 48 kHz |
|------|--------------|---------------------|---------------|
| Standard | 64 BCLK/frame | 64 | 3.072 MHz |
| TDM4 | 128 BCLK/frame | 128 | 6.144 MHz |
| TDM8 | 256 BCLK/frame | 256 | 12.288 MHz |
| TDM16 | 512 BCLK/frame | 512 | 24.576 MHz |

Typical MCLK multiples: 256x (standard/TDM4), 512x (TDM8), 1024x (TDM16).

## Specifications

- Sample rates: any rate achievable by APLL (8 kHz to 192 kHz)
- Bit depths: 16, 24, or 32 bit
- Slot width: configurable (>= bits_per_sample), controls BCLK rate
- Data lines: 1 to 15 parallel outputs
- Modes: Standard I2S (2-slot), TDM4, TDM8, TDM16
- Total channels: up to 240 (15 lines x 16 TDM slots)
- Format: I2S Philips (standard) or TDM with 1-BCLK frame sync
- All channels share MCLK/BCLK/LRCK -- zero inter-channel clock drift

## Latency

The output latency is determined by the DMA buffer pipeline:

```
Latency = frames_per_buffer * dma_buffer_count / sample_rate
```

| frames_per_buffer | dma_buffer_count | Latency @ 48 kHz |
|-------------------|-----------------|-------------------|
| 64 | 4 | 5.3 ms |
| 128 | 4 | 10.7 ms |
| 32 | 3 | 2.0 ms |
| 16 | 2 | 0.67 ms |

The minimum practical latency depends on how fast your application can fill DMA buffers. With smaller buffers, the CPU has less time to prepare the next chunk before an underrun occurs. For real-time audio processing at 48 kHz, 2-4 ms (32-64 frames, 2-4 buffers) is a reasonable target.

There is no additional latency from clock synchronization -- BCLK, LRCK, and all data lines are output from the same PARLIO clock domain in the same DMA cycle, so all signals are phase-aligned to within a single MCLK period (< 100 ns at typical audio rates).

## GPIO Wiring

| Signal | Default GPIO | PARLIO Function | Description |
|--------|-------------|-----------------|-------------|
| MCLK | 10 | ext clk input | I2S MCLK output + PARLIO clock source |
| BCLK | 11 | clk_out (dedicated) | Bit clock, does not use a data line |
| LRCK | 12 | TXD[0] | Word select (I2S) or frame sync (TDM) |
| DATA0 | 13 | TXD[1] | Data line 0 |
| DATA1 | 14 | TXD[2] | Data line 1 |

The MCLK GPIO serves double duty: it outputs MCLK from the I2S/APLL (usable by DACs that need MCLK) and simultaneously feeds the PARLIO peripheral as external clock input via the GPIO matrix. No external loopback wire is needed.

## Use Cases

### Multi-DAC Audio Systems
Drive multiple I2S DAC chips from a single peripheral. With 15 data lines, you can feed up to 15 stereo DACs simultaneously with perfectly synchronized clocks -- useful for immersive audio installations, Ambisonics rigs, or line array speaker processors.

### High Channel Count Outputs
Build a 240-channel audio output stage using TDM16 with 15 data lines. All channels share the same MCLK/BCLK/FSYNC, eliminating inter-channel clock drift that plagues multi-peripheral I2S setups.

### Overcoming I2S Peripheral Limitations
The ESP32-P4 has 3 I2S peripherals. If your design needs more than 3 independent I2S outputs, or needs more than the native TDM slot count, PARLIO gives you additional outputs without consuming I2S hardware (only one I2S channel is used as a clock source).

### Custom Digital Audio Protocols
Since LRCK and data are software-defined in the DMA buffer and slot_width is freely configurable, you can implement non-standard framing: asymmetric slot widths, custom LRCK patterns, DSP/PCM short/long frame sync, or TDM with arbitrary slot counts per data line.

### Deterministic Multi-Channel Test Signal Generation
Generate precisely timed test signals across many channels simultaneously -- useful for production-line testing of audio hardware, speaker arrays, or codec verification.

### ADAT/SPDIF Bitstream Generation
With software-defined output and configurable bit rates, you could synthesize ADAT lightpipe or SPDIF bitstreams directly, without dedicated transmitter ICs.

## S/PDIF Transmitter

Single-wire S/PDIF output using biphase mark coding (BMC). Uses PARLIO in 1-bit serial mode at 128 * Fs.

- Stereo, 16/20/24 bit
- Consumer or professional channel status
- 192-frame channel status block transmitted cyclically
- One GPIO pin for the S/PDIF output (directly drives coax or optical transmitter)

```c
parlio_spdif_tx_config_t cfg = {
    .sample_rate = 48000,
    .bits_per_sample = 24,
    .mclk_multiple = 256,
    .mclk_gpio = GPIO_NUM_20,
    .spdif_gpio = GPIO_NUM_22,
    .consumer_format = true,
};
parlio_spdif_tx_handle_t spdif;
parlio_spdif_tx_new(&cfg, &spdif);
parlio_spdif_tx_enable(spdif);

int32_t stereo[2] = { left_sample, right_sample };
parlio_spdif_tx_write(spdif, stereo, 1, NULL, 1000);
```

## ADAT Lightpipe Transmitter

8-channel ADAT output using NRZI encoding. Uses PARLIO in 1-bit serial mode at 256 * Fs.

- 8 channels, 24-bit, 48 kHz (standard ADAT)
- One GPIO pin for the ADAT output (drives TOTX173/177 optical transmitter or coax)
- Frame includes sync pattern, nibble separators, and per-channel user bits

```c
parlio_adat_tx_config_t cfg = {
    .sample_rate = 48000,
    .mclk_multiple = 512,
    .mclk_gpio = GPIO_NUM_20,
    .adat_gpio = GPIO_NUM_22,
};
parlio_adat_tx_handle_t adat;
parlio_adat_tx_new(&cfg, &adat);
parlio_adat_tx_enable(adat);

int32_t frame[8] = { ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7 };
parlio_adat_tx_write(adat, frame, 1, NULL, 1000);
```

## Unified Multi-Protocol Output

The unified driver (`parlio_audio_tx.h`) can output I2S, S/PDIF, and ADAT simultaneously on a single PARLIO TX unit. All protocols are encoded as parallel bit patterns in the same DMA buffer and shift out in lockstep.

The PARLIO clock runs at the fastest protocol's rate (256 * Fs when ADAT is active). Slower protocols repeat their symbols to match. All outputs are sample-locked with zero inter-protocol skew.

```c
parlio_audio_i2s_config_t i2s_cfg = {
    .mode = PARLIO_AUDIO_I2S_STANDARD,
    .bits_per_sample = 32, .slot_width = 32, .num_data_lines = 4,
    .bclk_gpio = GPIO_NUM_22, .lrck_gpio = GPIO_NUM_23,
    .data_gpios = { GPIO_NUM_24, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27 },
};
parlio_audio_spdif_config_t spdif_cfg = {
    .bits_per_sample = 24, .consumer_format = true,
    .spdif_gpio = GPIO_NUM_28,
};
parlio_audio_adat_config_t adat_cfg = {
    .adat_gpio = GPIO_NUM_29,
};

parlio_audio_tx_config_t cfg = {
    .sample_rate = 48000,
    .mclk_gpio = GPIO_NUM_20,
    .clk_out_gpio = GPIO_NUM_21,    /* outputs PARLIO clock as MCLK */
    .i2s = &i2s_cfg,                /* 4 lines x 2 = 8 I2S channels */
    .spdif = &spdif_cfg,            /* 2 S/PDIF channels */
    .adat = &adat_cfg,              /* 8 ADAT channels */
};

parlio_audio_tx_handle_t tx;
parlio_audio_tx_new(&cfg, &tx);
parlio_audio_tx_enable(tx);

/* Write all 18 channels per frame: [8x I2S, 2x SPDIF, 8x ADAT] */
size_t frame_size = parlio_audio_tx_get_frame_size(tx); /* = 18 */
int32_t samples[18] = { ... };
parlio_audio_tx_write(tx, samples, 1, NULL, 1000);
```

### I2S HW Passthrough

The I2S peripheral used for APLL clock generation is normally idle (only MCLK output). With the `i2s_hw` config, it also outputs real audio on its own BCLK/WS/DOUT pins -- zero additional peripheral cost. Write to it via the standard ESP-IDF `i2s_channel_write()` API.

```c
parlio_audio_i2s_hw_config_t hw_cfg = {
    .bits_per_sample = 32,
    .total_slots = 8,                   /* TDM8 on the I2S HW peripheral */
    .bclk_gpio = GPIO_NUM_30,
    .ws_gpio   = GPIO_NUM_31,
    .dout_gpio = GPIO_NUM_32,
    .dout2_gpio = -1,                   /* second DOUT available on I2S0 only */
};

parlio_audio_tx_config_t cfg = {
    .sample_rate = 48000,
    .mclk_gpio = GPIO_NUM_20,
    .adat = &adat_cfg,                  /* 8 ch ADAT via PARLIO */
    .i2s_hw = &hw_cfg,                  /* 8 ch TDM via I2S HW */
};

parlio_audio_tx_handle_t tx;
parlio_audio_tx_new(&cfg, &tx);
parlio_audio_tx_enable(tx);

/* Write ADAT via PARLIO path */
parlio_audio_tx_write(tx, adat_samples, num_frames, NULL, 1000);

/* Write I2S HW via standard ESP-IDF API (sample-locked, same APLL) */
i2s_chan_handle_t i2s_hw = parlio_audio_tx_get_i2s_hw_handle(tx);
i2s_channel_write(i2s_hw, tdm_samples, size, &bytes_written, 1000);
```

### Resource usage per combination

| Configuration | PARLIO clock | Data width | DMA/frame | PARLIO ch | I2S HW ch | Total |
|---------------|-------------|------------|-----------|-----------|-----------|-------|
| I2S only (4 lines) | 64*Fs | 8-bit | 64 B | 8 | -- | 8 |
| ADAT only | 256*Fs | 1-bit | 32 B | 8 | -- | 8 |
| ADAT + I2S HW TDM8 | 256*Fs | 1-bit | 32 B | 8 | 8 | 16 |
| I2S (4) + SPDIF + ADAT | 256*Fs | 8-bit | 256 B | 18 | -- | 18 |
| I2S (4) + SPDIF + ADAT + I2S HW TDM16 | 256*Fs | 8-bit | 256 B | 18 | 16 | 34 |
| I2S (12) + SPDIF + ADAT + I2S HW TDM16x2 | 256*Fs | 16-bit | 512 B | 34 | 32 | 66 |
| Max (all I2S peripherals) | 256*Fs | 16-bit | 512 B | 34 | 64 | 98 |

## Build

```bash
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

## Demo Application

The included `main.c` demonstrates the unified driver outputting three protocols simultaneously:

| Output | Protocol | Channels | Content |
|--------|----------|----------|---------|
| PARLIO TXD[0-2] | I2S Philips | 2 (stereo) | 440 Hz / 880 Hz sine |
| PARLIO TXD[3] | ADAT | 8 | 200-900 Hz across channels |
| I2S HW (GPIO 27-29) | I2S Philips | 2 (stereo) | 1000 Hz / 1500 Hz sine |
| **Total** | | **12 channels** | All sample-locked |

Requires a physical wire between GPIO 20 and GPIO 21 (MCLK loopback). Runs a 30-second timed test reporting effective sample rate, then loops continuously for scope inspection.

GPIO assignments in the demo:
- GPIO 20: MCLK out (wire source)
- GPIO 21: PARLIO ext clk in (wire destination)
- GPIO 22: PARLIO clk_out (256*Fs, usable as MCLK)
- GPIO 23: I2S BCLK (PARLIO TXD[0], synthesized)
- GPIO 24: I2S LRCK (PARLIO TXD[1])
- GPIO 25: I2S DATA (PARLIO TXD[2])
- GPIO 26: ADAT output (PARLIO TXD[3])
- GPIO 27-29: I2S HW BCLK/WS/DOUT

## Component API

### Standalone drivers (single protocol, simple)

- `parlio_i2s.h` -- I2S/TDM transmitter (BCLK on clk_out, efficient)
- `parlio_spdif_tx.h` -- S/PDIF transmitter (1-bit serial, BMC)
- `parlio_adat_tx.h` -- ADAT transmitter (1-bit serial, NRZI)

### Unified driver (multi-protocol, flexible)

- `parlio_audio_tx.h` -- any combination of I2S + S/PDIF + ADAT + I2S HW passthrough

Key functions:
- `parlio_audio_tx_new()` -- configure protocols, auto-select clock rates
- `parlio_audio_tx_enable()` -- start all outputs with silence pre-fill
- `parlio_audio_tx_write()` -- write interleaved samples for all PARLIO protocols
- `parlio_audio_tx_get_i2s_hw_handle()` -- get I2S channel for HW passthrough writes
- `parlio_audio_tx_get_frame_size()` -- query samples per frame
- `parlio_audio_tx_delete()` -- clean up

### Quick Example (TDM8, 2 lines = 16 channels, standalone)

```c
parlio_i2s_tx_config_t cfg = {
    .sample_rate    = 48000,
    .bits_per_sample = 32,
    .slot_width     = 32,
    .num_data_lines = 2,
    .mclk_multiple  = 512,
    .mode           = PARLIO_I2S_MODE_TDM8,
    .mclk_gpio      = GPIO_NUM_10,
    .bclk_gpio      = GPIO_NUM_11,
    .lrck_gpio      = GPIO_NUM_12,
    .data_gpios     = { GPIO_NUM_13, GPIO_NUM_14 },
    .dma_buffer_count  = 4,
    .frames_per_buffer = 64,
};

parlio_i2s_tx_handle_t tx;
parlio_i2s_tx_new(&cfg, &tx);
parlio_i2s_tx_enable(tx);

int32_t samples[16];
size_t written;
parlio_i2s_tx_write(tx, samples, 1, &written, 1000);
```
