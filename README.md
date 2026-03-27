# PARLIO I2S/TDM Transmitter for ESP32-P4

Multi-channel I2S/TDM transmitter that abuses the Parallel IO peripheral on the ESP32-P4 to synthesize I2S output signals (MCLK, BCLK, LRCK/FSYNC, variable data lines), clocked by the APLL for sample-accurate audio timing.

## How it works

The ESP32-P4's PARLIO peripheral is a parallel data output engine with DMA support, designed for display interfaces. We repurpose it as a multi-channel I2S/TDM transmitter:

1. **APLL** generates a precise audio master clock (e.g., 12.288 MHz for 48 kHz / 256x)
2. The **I2S peripheral** is configured purely as a clock generator -- it routes APLL to its MCLK output on a GPIO pin
3. That GPIO feeds back into **PARLIO as external clock input**
4. PARLIO outputs:
   - **MCLK** on its clock output pin (mirrors the ext clock)
   - **BCLK** as data bit 0 (synthesized in DMA buffer)
   - **LRCK / FSYNC** as data bit 1 (synthesized in DMA buffer)
   - **DATA[0..N]** as data bits 2+ (audio samples, MSB first)

Each MCLK tick, PARLIO shifts out one parallel word containing the instantaneous state of all output signals. The DMA buffer contains the pre-computed bit patterns for BCLK toggling, LRCK/FSYNC framing, and serialized audio data.

## Supported Modes

| Mode | Slots/Line | LRCK Behavior | Typical Use |
|------|-----------|---------------|-------------|
| `PARLIO_I2S_MODE_STANDARD` | 2 | 50/50 duty cycle (Philips I2S) | Stereo DACs, codecs |
| `PARLIO_I2S_MODE_TDM4` | 4 | 1-BCLK frame sync pulse | 4-ch DACs, small arrays |
| `PARLIO_I2S_MODE_TDM8` | 8 | 1-BCLK frame sync pulse | Multi-ch audio interfaces |
| `PARLIO_I2S_MODE_TDM16` | 16 | 1-BCLK frame sync pulse | High-density systems |

## Channel Counts

Each PARLIO data line carries `num_slots` audio channels. With up to 14 data lines available:

| Mode | Channels/Line | Max Lines | Max Channels |
|------|--------------|-----------|-------------|
| Standard I2S | 2 | 14 | **28** |
| TDM4 | 4 | 14 | **56** |
| TDM8 | 8 | 14 | **112** |
| TDM16 | 16 | 14 | **224** |

### MCLK Requirements per Mode

The MCLK multiple (`mclk_multiple`) must satisfy:
- `mclk_multiple >= slot_width * num_slots * 2` (MCLK/BCLK ratio must be at least 2)
- `mclk_multiple % (slot_width * num_slots) == 0` (must divide evenly)

| Mode | slot_width=32 | Minimum `mclk_multiple` | MCLK @ 48 kHz |
|------|--------------|------------------------|---------------|
| Standard | 32 x 2 = 64 BCLK/frame | 128 | 6.144 MHz |
| TDM4 | 32 x 4 = 128 BCLK/frame | 256 | 12.288 MHz |
| TDM8 | 32 x 8 = 256 BCLK/frame | 512 | 24.576 MHz |
| TDM16 | 32 x 16 = 512 BCLK/frame | 1024 | 49.152 MHz |

## Specifications

- Sample rates: any rate achievable by APLL (8 kHz to 192 kHz)
- Bit depths: 16, 24, or 32 bit
- Slot width: configurable (>= bits_per_sample, typically 32)
- Data lines: 1 to 14 parallel outputs
- Modes: Standard I2S (2-slot), TDM4, TDM8, TDM16
- Total channels: up to 224 (14 lines x 16 TDM slots)
- Format: I2S Philips (standard) or TDM with 1-BCLK frame sync
- All channels share MCLK/BCLK/LRCK -- zero inter-channel clock drift

## GPIO Wiring

| Signal | Default GPIO | Description |
|--------|-------------|-------------|
| MCLK | 10 | Master clock output to DAC |
| APLL feed | 11 | I2S MCLK loopback into PARLIO ext clock |
| BCLK | 12 | Bit clock |
| LRCK / FSYNC | 13 | Word select (I2S) or frame sync (TDM) |
| DATA0 | 14 | Data line 0 |
| DATA1 | 15 | Data line 1 |

The APLL feed GPIO requires a short physical wire or PCB trace connecting the I2S MCLK output back to the PARLIO external clock input. On some board layouts this can be adjacent pins.

## Use Cases

### Multi-DAC Audio Systems
Drive multiple I2S DAC chips from a single peripheral. With 14 data lines, you can feed up to 14 stereo DACs simultaneously with perfectly synchronized clocks -- useful for immersive audio installations, Ambisonics rigs, or line array speaker processors.

### High Channel Count Recording Interfaces
Build a 224-channel audio output stage using TDM16 with 14 data lines. All channels share the same MCLK/BCLK/FSYNC, eliminating inter-channel clock drift that plagues multi-peripheral I2S setups.

### Overcoming I2S Peripheral Limitations
The ESP32-P4 has 3 I2S peripherals. If your design needs more than 3 independent I2S outputs, or needs more than the native TDM slot count, PARLIO gives you additional outputs without consuming I2S hardware (only one I2S channel is used as a clock source).

### Custom Digital Audio Protocols
Since BCLK, LRCK, and data are all software-defined in the DMA buffer, you can implement non-standard framing: asymmetric slot widths, custom LRCK patterns, DSP/PCM short/long frame sync, or even TDM with arbitrary slot counts per data line.

### Deterministic Multi-Channel Test Signal Generation
Generate precisely timed test signals across many channels simultaneously -- useful for production-line testing of audio hardware, speaker arrays, or codec verification.

### ADAT/SPDIF Bitstream Generation
With MCLK rates up to ~40 MHz (limited by PARLIO max clock) and fully software-defined output, you could synthesize ADAT lightpipe or SPDIF bitstreams directly, without dedicated transmitter ICs.

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
    .slot_width     = 32,
    .num_data_lines = 2,
    .mclk_multiple  = 512,
    .mode           = PARLIO_I2S_MODE_TDM8,
    .mclk_gpio      = GPIO_NUM_10,
    .apll_feed_gpio = GPIO_NUM_11,
    .bclk_gpio      = GPIO_NUM_12,
    .lrck_gpio      = GPIO_NUM_13,
    .data_gpios     = { GPIO_NUM_14, GPIO_NUM_15 },
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
