# PARLIO I2S Transmitter for ESP32-P4

Multi-channel I2S transmitter that abuses the Parallel IO peripheral on the ESP32-P4 to synthesize I2S signals (MCLK, BCLK, LRCK, variable data lines), clocked by the APLL for sample-accurate audio timing.

## How it works

The ESP32-P4's PARLIO peripheral is a parallel data output engine with DMA support, designed for display interfaces. We repurpose it as a multi-channel I2S transmitter:

1. **APLL** generates a precise audio master clock (e.g., 12.288 MHz for 48 kHz / 256x)
2. The **I2S peripheral** is configured purely as a clock generator -- it routes APLL to its MCLK output on a GPIO pin
3. That GPIO feeds back into **PARLIO as external clock input**
4. PARLIO outputs:
   - **MCLK** on its clock output pin (mirrors the ext clock)
   - **BCLK** as data bit 0 (synthesized in DMA buffer)
   - **LRCK** as data bit 1 (synthesized in DMA buffer)
   - **DATA[0..N]** as data bits 2+ (audio samples, MSB first)

Each MCLK tick, PARLIO shifts out one parallel word containing the instantaneous state of all output signals. The DMA buffer contains the pre-computed bit patterns for BCLK toggling, LRCK framing, and serialized audio data.

## Specifications

- Sample rates: any rate achievable by APLL (8 kHz to 192 kHz)
- Bit depths: 16, 24, or 32 bit
- Slot width: configurable (>= bits_per_sample, typically 32)
- Data lines: 1 to 14 parallel outputs (each carries one stereo pair)
- Total channels: up to 28 (14 lines x 2 channels)
- MCLK multiple: configurable (typically 256x or 384x)
- Format: I2S Philips (MSB first, 1-bit LRCK offset)

## GPIO Wiring

| Signal | Default GPIO | Description |
|--------|-------------|-------------|
| MCLK | 10 | Master clock output to DAC |
| APLL feed | 11 | I2S MCLK loopback into PARLIO ext clock |
| BCLK | 12 | Bit clock |
| LRCK | 13 | Word select / frame sync |
| DATA0 | 14 | Stereo pair 0 (L0/R0) |
| DATA1 | 15 | Stereo pair 1 (L1/R1) |

The APLL feed GPIO requires a short physical wire or PCB trace connecting the I2S MCLK output back to the PARLIO external clock input. On some board layouts this can be adjacent pins.

## Use Cases

### Multi-DAC Audio Systems
Drive multiple I2S DAC chips from a single peripheral. With 14 data lines, you can feed up to 14 stereo DACs simultaneously with perfectly synchronized clocks -- useful for immersive audio installations, Ambisonics rigs, or line array speaker processors.

### High Channel Count Recording Interfaces
Build a 28-channel audio output stage for studio or live sound applications. All channels share the same MCLK/BCLK/LRCK, eliminating inter-channel clock drift that plagues multi-peripheral I2S setups.

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

- `parlio_i2s_tx_new()` -- create and configure
- `parlio_i2s_tx_enable()` -- start clocks and pre-fill silence
- `parlio_i2s_tx_write()` -- write interleaved int32_t audio frames
- `parlio_i2s_tx_disable()` -- stop output
- `parlio_i2s_tx_delete()` -- free resources
