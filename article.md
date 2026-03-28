# I Made an ESP32-P4 Output 304 Channels of Audio by Abusing a Display Peripheral

*How I turned a parallel display interface into the most capable audio output stage ever seen on a microcontroller -- and tested 176 channels at 48 kHz on real hardware*

The ESP32-P4 has three I2S peripherals. Each supports up to 2 data output pins with 16 TDM slots. That gives you a theoretical maximum of 96 audio channels if you use all three. Respectable, but I wanted more.

So I ignored the I2S peripherals entirely and built an audio transmitter out of PARLIO -- a peripheral designed for driving LCD panels and LED matrices. The architecture supports up to **304 channels** (240 via PARLIO + 64 via the I2S peripherals riding along for free). I've tested **176 channels of simultaneous 48 kHz, 32-bit audio** on real hardware, verified cycle-perfect using the chip's built-in pulse counter.

Here's how and why.

## The Problem

I'm building a beamforming system on the ESP32-P4. Beamforming -- where you use an array of speakers with individually delayed and weighted signals to steer sound in specific directions -- requires a lot of audio output channels. The more channels, the tighter the beam, the better the spatial resolution.

The P4's I2S peripherals top out at 32 channels per port (TDM16 with 2 data pins). Even using all three I2S ports, you're at 96 channels max, and they don't share a clock -- meaning potential inter-channel drift between ports.

I needed:

- More channels (100+)
- All from the same clock source (zero drift between any channels)
- Standard I2S/TDM framing (compatible with off-the-shelf DAC chips)
- 48 kHz, 32-bit, real-time, no gaps

## The Insight: PARLIO Has 16 Parallel Data Lines

The ESP32-P4 includes a Parallel IO (PARLIO) peripheral meant for driving RGB LED matrices and parallel display interfaces. It's basically a DMA-fed shift register: on every clock tick, it shifts out one word across up to 16 parallel data lines simultaneously.

    PARLIO TX peripheral:
      clk_out  ---------> output clock (dedicated pin)
      TXD[0]   ---------> parallel data line 0
      TXD[1]   ---------> parallel data line 1
      ...
      TXD[15]  ---------> parallel data line 15

      Each clock tick: one 16-bit word shifted out across all lines
      Data source: DMA from memory (zero CPU involvement during output)

16 parallel data lines. If I encode I2S signals into those lines, that's 15 data outputs (one line for LRCK/frame sync) times 16 TDM slots = **240 channels**. From one peripheral.

## The Trick: Three Peripherals, One Clock

The challenge: PARLIO doesn't have an audio-grade clock source. It supports XTAL (40 MHz), PLL (160 MHz), and external clock input -- but not the APLL (Audio PLL) that generates precise frequencies like 12.288 MHz for 48 kHz audio.

My solution chains three peripherals together in ways they were never intended:

    APLL           I2S peripheral        GPIO matrix         PARLIO peripheral
    (audio PLL)    (clock only!)         (loopback)          (parallel shift out)

    +-----------+  +--------------+     +-----------+       +-----------------+
    | 12.288    |->| MCLK output  |---->| Same GPIO |------>| Ext clock in    |
    | MHz       |  | (no audio!)  |     | is both   |       | Divides to BCLK |
    +-----------+  +--------------+     | out & in  |       | Shifts DMA data |
                                        +-----------+       +-----------------+

**APLL** generates the precise audio master clock. **I2S** is allocated purely to route APLL to a GPIO pin -- we never send audio through it. The **GPIO matrix** feeds that same pin back into PARLIO as an external clock input. PARLIO divides it down to BCLK and shifts out pre-encoded bit patterns from DMA buffers.

The I2S peripheral -- one of the most capable audio interfaces on the chip -- is reduced to a clock conduit. It has no idea it's feeding a display peripheral that's pretending to be an audio interface.

## Encoding I2S in a Parallel Shift Register

The DMA buffer contains one byte per BCLK cycle. Each bit position in the byte maps to a physical output pin:

    Byte layout per BCLK tick:
      bit 0 = LRCK (word select / frame sync)
      bit 1 = audio data line 0
      bit 2 = audio data line 1
      ...
      bit 7 = audio data line 6

    For stereo 32-bit on one data line:
      BCLK  0: 0x00  (LRCK=0, left slot padding)
      BCLK  1: 0x02  (LRCK=0, MSB of left sample = 1)
      BCLK  2: 0x00  (LRCK=0, next bit = 0)
      ...
      BCLK 32: 0x01  (LRCK=1, right slot padding)
      BCLK 33: 0x03  (LRCK=1, MSB of right sample = 1)

BCLK itself comes from PARLIO's dedicated clock output pin -- it doesn't consume a data line. The clock output is the PARLIO shift clock, which is exactly BCLK after the internal divider.

## The LUT Encoder: 4 Table Lookups Per Sample

The naive approach -- extracting one bit at a time from each audio sample (31 shift-mask-store operations per channel) -- was too slow for high channel counts at 48 kHz. At TDM8 with 2 data lines (16 channels), the bit-by-bit encoder hit only 92.6% of real-time.

The fix: a precomputed lookup table. For each data line position, I build a 256-entry table that maps one byte of a sample to 8 output bytes:

    line_lut[line][byte_value] = 8 output bytes packed as uint64_t

    Example: line 0 (bit position 1), input byte 0xA5 = 10100101:
    line_lut[0][0xA5] = { 0x02, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x02 }

A 32-bit sample has 4 bytes. Four LUT lookups plus four uint64_t writes produce 32 output bytes. Multiple data lines: OR the LUT results together. This is roughly 4x faster than bit-by-bit extraction and brought all configurations up to full speed.

## Gapless DMA: Loop Transmission

Even with the fast encoder, there was a persistent 0.3% throughput gap. The PARLIO driver uses one-shot DMA: each buffer is a separate transfer. When one finishes, the ISR stops the peripheral, dequeues the next transfer, reconfigures DMA, and restarts. That stop-start cycle takes about 16 microseconds -- enough to cause audible glitches at 48 kHz.

The solution: PARLIO's loop\_transmission mode. When you submit a buffer with loop enabled, the DMA loops it continuously. Submitting a new buffer with the same flag does a hot-swap: the DMA hardware chains to the new buffer via descriptor concatenation without ever stopping. Zero gap.

I use three rotating buffers: while one plays, the encoder fills the next. Submissions are paced using microsecond-precision timing derived from the total frame count to prevent drift.

## Hardware Verification: Pulse Counter on the Wire

Software frame counters measure encoder throughput, not actual output frequency. To prove the output is exactly 48,000 Hz, I used the ESP32-P4's PCNT (Pulse Counter) peripheral to count rising edges of the BCLK signal directly on the output GPIO:

    Configuration             Target   Ch    Measured    Error
    ---------------------------------------------------------
    Stereo x1 (2ch)           48000    2    47999.9 Hz  -2ppm   PASS
    TDM4 x11 (44ch)           48000   44    48000.0 Hz  -0ppm   PASS
    TDM8 x11 (88ch)           48000   88    48000.0 Hz  -0ppm   PASS
    TDM16 x7 (112ch)          48000  112    48000.0 Hz  -0ppm   PASS
    TDM16 x11 (176ch)         48000  176    48000.0 Hz  -0ppm   PASS
    Stereo 192kHz             192000    2   191999.9 Hz  -0ppm   PASS
    Stereo 44.1kHz             44100    2    44100.0 Hz  -0ppm   PASS

176 channels at 48 kHz, 32-bit, with sub-ppm accuracy. Every channel sharing the same APLL-derived clock. Hardware-verified on the wire.

## What's Next: Beamforming and ADAT on ESP32-P4

This audio output stage is the foundation for a real-time beamforming system.

**Phase 1: MEMS microphone array input.** The ESP32-P4's I2S RX peripherals can capture PDM or TDM input from arrays of MEMS microphones. Combined with the PARLIO output, the same chip handles both capture and playback.

**Phase 2: Real-time DSP.** The P4 runs at 360 MHz with hardware floating point. Per-channel delay, gain, and FIR filtering for beamforming weights. The dual-core architecture lets one core handle DSP while the other feeds the PARLIO encoder.

**Phase 3: ADAT output.** The same PARLIO peripheral can also output ADAT Lightpipe (8 channels of 24-bit audio over a single optical fiber) using 1-bit NRZI encoding at 256x the sample rate. I've already built standalone ADAT and S/PDIF encoders that share the PARLIO infrastructure. A unified driver can output I2S, ADAT, and S/PDIF simultaneously from the same DMA buffer.

## The Numbers

- **Max channels (PARLIO only):** 240 (15 lines x TDM16)
- **Max channels (+ I2S HW):** 304 (add 3 I2S peripherals as bonus outputs)
- **Tested on hardware:** 176 channels at 48 kHz, 32-bit
- **Sample rates tested:** 8 kHz to 192 kHz
- **Clock accuracy:** less than 2 ppm (APLL + PCNT verified)
- **Output latency:** 2 to 8 ms (configurable buffer size)
- **Encoder throughput:** about 32x real-time (LUT-based)
- **DMA gap:** zero (loop transmission mode)
- **CPU core used:** 1 of 2 (dedicated encoder task on CPU1)
- **Peripherals abused:** 3 (APLL, I2S, PARLIO)

## Try It Yourself

The full implementation is open source:

[github.com/DatanoiseTV/i2s-parlio-esp32p4](https://github.com/DatanoiseTV/i2s-parlio-esp32p4)

It includes:

- **parlio\_i2s.h** -- Standalone I2S/TDM driver with LUT encoder and loop DMA
- **parlio\_spdif\_tx.h** -- S/PDIF transmitter (biphase mark coding on a single wire)
- **parlio\_adat\_tx.h** -- ADAT Lightpipe transmitter (8-channel NRZI optical output)
- **parlio\_audio\_tx.h** -- Unified multi-protocol driver (I2S + S/PDIF + ADAT simultaneously)
- Comprehensive test suite with PCNT hardware verification across 35 configurations

Requires an ESP32-P4 board, ESP-IDF v5.4+, and a wire between two GPIO pins.

*Built on the ESP32-P4-NANO. The PARLIO peripheral was designed to drive LED matrices. It had no idea what was coming.*
