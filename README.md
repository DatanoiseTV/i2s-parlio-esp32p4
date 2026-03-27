# PARLIO I2S/TDM Transmitter for ESP32-P4

Multi-channel I2S/TDM transmitter that abuses the Parallel IO peripheral on the ESP32-P4 to synthesize I2S output signals (MCLK, BCLK, LRCK/FSYNC, variable data lines), clocked by the APLL for sample-accurate audio timing.

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

## Build

```bash
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

## Component API

See `components/parlio_i2s/include/parlio_i2s.h` for the full API:

- `parlio_i2s_tx_new()` -- create and configure (supports all modes)
- `parlio_i2s_tx_enable()` -- start clocks and pre-fill silence
- `parlio_i2s_tx_write()` -- write interleaved int32_t audio frames
- `parlio_i2s_tx_disable()` -- stop output
- `parlio_i2s_tx_delete()` -- free resources

### Quick Example (TDM8, 2 lines = 16 channels)

```c
parlio_i2s_tx_config_t cfg = {
    .sample_rate    = 48000,
    .bits_per_sample = 32,
    .slot_width     = 32,          /* 32 BCLK per slot, 256 BCLK per TDM8 frame */
    .num_data_lines = 2,
    .mclk_multiple  = 512,         /* MCLK = 24.576 MHz, MCLK/BCLK = 2 */
    .mode           = PARLIO_I2S_MODE_TDM8,
    .mclk_gpio      = GPIO_NUM_10, /* MCLK out + PARLIO clk in */
    .bclk_gpio      = GPIO_NUM_11, /* BCLK on dedicated clk_out */
    .lrck_gpio      = GPIO_NUM_12, /* frame sync on TXD[0] */
    .data_gpios     = { GPIO_NUM_13, GPIO_NUM_14 },
    .dma_buffer_count  = 4,
    .frames_per_buffer = 64,
};

parlio_i2s_tx_handle_t tx;
parlio_i2s_tx_new(&cfg, &tx);
parlio_i2s_tx_enable(tx);

/* Write 16-channel interleaved audio:
 * [line0_slot0, line0_slot1, ..., line0_slot7,
 *  line1_slot0, line1_slot1, ..., line1_slot7] per frame */
int32_t samples[16];
size_t written;
parlio_i2s_tx_write(tx, samples, 1, &written, 1000);
```
