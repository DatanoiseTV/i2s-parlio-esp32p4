#include "parlio_audio_tx.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/parlio_tx.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "parlio_audio";

/* ------------------------------------------------------------------ */
/*  Per-line LUT (shared with standalone driver)                       */
/* ------------------------------------------------------------------ */

#define MAX_LUT_LINES 15
static DRAM_ATTR uint64_t s_line_lut[MAX_LUT_LINES][256];
static DRAM_ATTR uint64_t s_lrck_mask_64; /* 0x0101010101010101 */
static int s_lut_lines = 0;

static void ensure_line_luts(int num_lines)
{
    if (num_lines <= s_lut_lines) return;
    if (num_lines > MAX_LUT_LINES) num_lines = MAX_LUT_LINES;
    for (int line = s_lut_lines; line < num_lines; line++) {
        int bit_pos = 1 + line; /* TXD position for this data line */
        for (int v = 0; v < 256; v++) {
            uint8_t out[8];
            for (int b = 0; b < 8; b++)
                out[b] = ((v >> (7 - b)) & 1) << bit_pos;
            memcpy(&s_line_lut[line][v], out, 8);
        }
    }
    s_lrck_mask_64 = 0x0101010101010101ULL;
    s_lut_lines = num_lines;
}

/* ------------------------------------------------------------------ */
/*  Internal sub-structures for each protocol                          */
/* ------------------------------------------------------------------ */

/* S/PDIF preamble tables (BMC-encoded, 8 UI each) */
static const uint8_t SPDIF_PRE_B[2] = { 0xE8, 0x17 }; /* block start, left */
static const uint8_t SPDIF_PRE_M[2] = { 0xE2, 0x1D }; /* left, not block start */
static const uint8_t SPDIF_PRE_W[2] = { 0xE4, 0x1B }; /* right */

/* Consumer channel status for 48 kHz, 24-bit, no copy protection */
static const uint8_t SPDIF_CS_CONSUMER[24] = {
    0x04, 0x00, 0x00, 0x02, 0x0B,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
static const uint8_t SPDIF_CS_PROFESSIONAL[24] = {
    0x01, 0x00, 0x00, 0x02, 0x00,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

typedef struct {
    uint8_t  data_bit;          /* TXD index */
    uint8_t  bits_per_sample;
    uint8_t  channel_status[24];
    uint16_t frame_in_block;    /* 0..191 */
    uint8_t  bmc_state;         /* running BMC level */
    uint16_t ticks_per_ui;      /* parlio ticks per BMC unit interval */
} spdif_state_t;

typedef struct {
    uint8_t  data_bit;          /* TXD index */
    uint8_t  nrzi_state;        /* running NRZI level */
    uint16_t ticks_per_bit;     /* parlio ticks per ADAT bit (1 when ADAT drives the clock) */
} adat_state_t;

typedef struct {
    uint8_t  bclk_bit;          /* TXD index for BCLK (unused if bclk_on_clk_out) */
    uint8_t  lrck_bit;          /* TXD index for LRCK */
    uint8_t  data_start_bit;    /* first TXD index for audio data */
    uint8_t  num_data_lines;
    uint8_t  slot_width;
    uint8_t  bits_per_sample;
    uint8_t  num_slots;
    parlio_audio_i2s_mode_t mode;
    uint16_t ticks_per_bclk;    /* parlio ticks per BCLK cycle */
    bool     bclk_on_clk_out;  /* true in I2S-only mode */
} i2s_state_t;

struct parlio_audio_tx {
    uint32_t sample_rate;
    uint32_t parlio_clock;
    uint16_t ticks_per_frame;   /* parlio_clock / sample_rate */
    uint8_t  parlio_width;      /* 1, 2, 4, 8, or 16 */
    uint16_t mclk_multiple;

    /* Protocol states (NULL if not configured) */
    i2s_state_t   *i2s;
    spdif_state_t *spdif;
    adat_state_t  *adat;

    /* Pre-computed per-tick lookup tables (eliminates ALL divisions in hot path).
     * Allocated once during init, ticks_per_frame entries each. */
    uint16_t *tick_static_mask;  /* BCLK + LRCK bits for each tick (constant every frame) */
    uint8_t  *tick_slot;         /* which slot (channel) this tick belongs to */
    int8_t   *tick_bit_idx;      /* audio bit index within the slot (-1 = no data) */
    uint16_t *tick_spdif_ui;     /* SPDIF unit interval index for each tick */
    uint16_t *tick_adat_bi;      /* ADAT bit index for each tick */

    /* Sample layout */
    size_t samples_per_frame;
    size_t i2s_offset;
    size_t spdif_offset;
    size_t adat_offset;

    /* DMA */
    size_t   frame_buf_bytes;
    size_t   dma_buf_bytes;
    size_t   frames_per_buf;
    size_t   dma_buf_count;
    uint8_t  **dma_bufs;
    size_t   write_buf_idx;
    size_t   write_frame_pos;

    SemaphoreHandle_t write_sem;
    i2s_chan_handle_t       i2s_clk_chan;
    bool                    i2s_hw_enabled;
    parlio_tx_unit_handle_t parlio_unit;
    int64_t  start_time_us;
    uint64_t total_submitted;
    bool loop_mode;
    bool enabled;
};

/* ------------------------------------------------------------------ */
/*  Utility                                                            */
/* ------------------------------------------------------------------ */

static uint8_t next_parlio_width(uint8_t needed)
{
    /* Always use at least 8-bit width to avoid sub-byte bit packing
     * in the encode loop. The unused TXD pins cost nothing. */
    if (needed <= 8)  return 8;
    if (needed <= 16) return 16;
    return 0;
}

static inline void write_parlio_word(uint8_t *dst, uint16_t idx,
                                      uint16_t word, uint8_t pw)
{
    if (pw <= 8) {
        dst[idx] = (uint8_t)word;
    } else {
        ((uint16_t *)dst)[idx] = word;
    }
}

/* ------------------------------------------------------------------ */
/*  S/PDIF encoder (frame -> 128-bit BMC buffer)                       */
/* ------------------------------------------------------------------ */

static inline uint8_t bmc_bit(uint8_t *buf, size_t *ui, uint8_t bit, uint8_t st)
{
    st ^= 1;
    buf[*ui / 8] |= (st << (*ui % 8));
    (*ui)++;
    if (bit) st ^= 1;
    buf[*ui / 8] |= (st << (*ui % 8));
    (*ui)++;
    return st;
}

static inline uint8_t bmc_preamble(uint8_t *buf, size_t *ui,
                                    const uint8_t pre[2], uint8_t st)
{
    uint8_t p = pre[st & 1];
    for (int i = 0; i < 8; i++) {
        buf[*ui / 8] |= (((p >> i) & 1) << (*ui % 8));
        (*ui)++;
    }
    return (p >> 7) & 1;
}

static void IRAM_ATTR encode_spdif_raw(spdif_state_t *s, uint8_t *out,
                               int32_t left, int32_t right)
{
    memset(out, 0, 16);
    size_t ui = 0;

    for (int sub = 0; sub < 2; sub++) {
        const uint8_t *pre;
        if (sub == 0)
            pre = (s->frame_in_block == 0) ? SPDIF_PRE_B : SPDIF_PRE_M;
        else
            pre = SPDIF_PRE_W;
        s->bmc_state = bmc_preamble(out, &ui, pre, s->bmc_state);

        uint32_t audio = (uint32_t)((sub == 0) ? left : right) >> 8;
        uint8_t parity = 0;
        for (int b = 0; b < 24; b++) {
            uint8_t v = (audio >> b) & 1;
            s->bmc_state = bmc_bit(out, &ui, v, s->bmc_state);
            parity ^= v;
        }
        s->bmc_state = bmc_bit(out, &ui, 0, s->bmc_state); /* validity */
        s->bmc_state = bmc_bit(out, &ui, 0, s->bmc_state); /* user data */

        uint8_t cs = 0;
        if (s->frame_in_block < 192) {
            uint8_t bi = s->frame_in_block / 8;
            if (bi < 24) cs = (s->channel_status[bi] >> (s->frame_in_block % 8)) & 1;
        }
        s->bmc_state = bmc_bit(out, &ui, cs, s->bmc_state);
        parity ^= cs;
        s->bmc_state = bmc_bit(out, &ui, parity, s->bmc_state); /* parity */
    }

    s->frame_in_block++;
    if (s->frame_in_block >= 192) s->frame_in_block = 0;
}

/* ------------------------------------------------------------------ */
/*  ADAT encoder (frame -> 256-bit NRZI buffer)                        */
/* ------------------------------------------------------------------ */

static inline void nrzi_bit(uint8_t *buf, size_t *idx, uint8_t bit, uint8_t *st)
{
    if (bit) *st ^= 1;
    if (*st) buf[*idx / 8] |= (1 << (*idx % 8));
    (*idx)++;
}

static void IRAM_ATTR encode_adat_raw(adat_state_t *s, uint8_t *out,
                              const int32_t *samples)
{
    memset(out, 0, 32);
    size_t bi = 0;

    /* Sync: 1 followed by 9 zeros */
    nrzi_bit(out, &bi, 1, &s->nrzi_state);
    for (int i = 0; i < 9; i++)
        nrzi_bit(out, &bi, 0, &s->nrzi_state);

    /* 8 channels */
    for (int ch = 0; ch < 8; ch++) {
        uint32_t audio = samples ? ((uint32_t)samples[ch] >> 8) : 0;

        nrzi_bit(out, &bi, 1, &s->nrzi_state); /* channel separator */

        for (int nib = 0; nib < 6; nib++) {
            int shift = 20 - nib * 4;
            for (int b = 3; b >= 0; b--)
                nrzi_bit(out, &bi, (audio >> (shift + b)) & 1, &s->nrzi_state);
            if (nib < 5)
                nrzi_bit(out, &bi, 1, &s->nrzi_state); /* nibble separator */
        }
    }

    /* Pad to 256 bits */
    while (bi < 256)
        nrzi_bit(out, &bi, 0, &s->nrzi_state);
}

/* ------------------------------------------------------------------ */
/*  Pre-compute per-tick lookup tables (called once during init)        */
/* ------------------------------------------------------------------ */

static void build_tick_tables(struct parlio_audio_tx *h)
{
    const uint16_t tpf = h->ticks_per_frame;

    h->tick_static_mask = calloc(tpf, sizeof(uint16_t));
    h->tick_slot        = calloc(tpf, sizeof(uint8_t));
    h->tick_bit_idx     = malloc(tpf * sizeof(int8_t));
    h->tick_spdif_ui    = calloc(tpf, sizeof(uint16_t));
    h->tick_adat_bi     = calloc(tpf, sizeof(uint16_t));
    if (!h->tick_static_mask || !h->tick_slot || !h->tick_bit_idx) return;

    memset(h->tick_bit_idx, -1, tpf);

    for (uint16_t tick = 0; tick < tpf; tick++) {
        uint16_t mask = 0;
        uint8_t slot = 0;
        int8_t bit_idx = -1;

        if (h->i2s) {
            i2s_state_t *is = h->i2s;
            uint16_t tpb = is->ticks_per_bclk;
            uint16_t bclk_idx = tick / tpb;
            uint16_t sub_tick = tick % tpb;

            if (!is->bclk_on_clk_out) {
                if (sub_tick < tpb / 2)
                    mask |= (1 << is->bclk_bit);
            }

            slot = bclk_idx / is->slot_width;
            if (is->mode == PARLIO_AUDIO_I2S_STANDARD) {
                if (slot >= 1) mask |= (1 << is->lrck_bit);
            } else {
                if (bclk_idx == 0) mask |= (1 << is->lrck_bit);
            }

            uint16_t pos_in_slot = bclk_idx % is->slot_width;
            int16_t bi = (int16_t)pos_in_slot - 1;
            if (bi >= 0 && bi < is->bits_per_sample)
                bit_idx = (int8_t)bi;
        }

        h->tick_static_mask[tick] = mask;
        h->tick_slot[tick] = slot;
        h->tick_bit_idx[tick] = bit_idx;

        /* Pre-compute SPDIF/ADAT bit indices (eliminates runtime divisions) */
        if (h->spdif)
            h->tick_spdif_ui[tick] = tick / h->spdif->ticks_per_ui;
        if (h->adat)
            h->tick_adat_bi[tick] = tick / h->adat->ticks_per_bit;
    }
}

/* ------------------------------------------------------------------ */
/*  Unified frame encoder (hot path, uses pre-computed tables)          */
/* ------------------------------------------------------------------ */

/*
 * Fast path: I2S-only encoder using LUT (same as standalone parlio_i2s).
 * Uses per-line LUT for 8-byte-at-a-time bit extraction. The LUT indices
 * are offset by data_start_bit since the unified driver may place data
 * lines at different TXD positions than the standalone driver.
 */
static void IRAM_ATTR encode_frame_i2s_only(struct parlio_audio_tx *h,
                                              uint8_t *dst,
                                              const int32_t *samples)
{
    const int32_t *samp = samples ? samples + h->i2s_offset : NULL;
    const i2s_state_t *is = h->i2s;
    const uint8_t sw  = is->slot_width;
    const uint8_t bps = is->bits_per_sample;
    const uint8_t ndl = is->num_data_lines;
    const uint8_t ns  = is->num_slots;
    const uint8_t dsb = is->data_start_bit;
    const bool    is_std = (is->mode == PARLIO_AUDIO_I2S_STANDARD);

    /* Build per-line LUTs indexed by TXD position (dsb + line) */
    /* s_line_lut is indexed 0..MAX_LUT_LINES-1 for data_start_bit positions */

    const uint8_t data_bits = (bps < sw) ? bps : (sw - 1);
    const uint8_t full_groups = data_bits / 8;
    const uint8_t remainder = data_bits % 8;

    if (!samp) {
        /* Silence: just LRCK pattern */
        memset(dst, 0, h->ticks_per_frame);
        if (is_std) {
            for (uint16_t p = sw; p < h->ticks_per_frame; p++)
                dst[p] = (1 << is->lrck_bit);
        } else {
            dst[0] = (1 << is->lrck_bit);
        }
        return;
    }

    for (uint8_t slot = 0; slot < ns; slot++) {
        uint16_t base = slot * sw;
        uint8_t lrck_byte = 0;
        if (is_std) {
            lrck_byte = (slot >= 1) ? (1 << is->lrck_bit) : 0;
        } else {
            lrck_byte = (slot == 0) ? (1 << is->lrck_bit) : 0;
        }

        dst[base] = lrck_byte;

        uint8_t *out = dst + base + 1;
        for (uint8_t g = 0; g < full_groups; g++) {
            uint64_t combined = 0;
            for (uint8_t line = 0; line < ndl; line++) {
                const uint8_t *sbytes = (const uint8_t *)&samp[line * ns + slot];
                combined |= s_line_lut[dsb - 1 + line][sbytes[3 - g]];
            }
            if (lrck_byte) {
                uint64_t lm = 0;
                memset(&lm, lrck_byte, 8); /* broadcast lrck_byte to all 8 bytes */
                combined |= lm;
            }
            memcpy(out + g * 8, &combined, 8);
        }

        if (remainder > 0) {
            uint64_t combined = 0;
            for (uint8_t line = 0; line < ndl; line++) {
                const uint8_t *sbytes = (const uint8_t *)&samp[line * ns + slot];
                combined |= s_line_lut[dsb - 1 + line][sbytes[3 - full_groups]];
            }
            if (lrck_byte) {
                uint64_t lm = 0;
                memset(&lm, lrck_byte, 8);
                combined |= lm;
            }
            uint8_t *src = (uint8_t *)&combined;
            for (uint8_t r = 0; r < remainder; r++)
                out[full_groups * 8 + r] = src[r];
        }

        uint16_t data_end = 1 + data_bits;
        for (uint16_t p = data_end; p < sw; p++)
            dst[base + p] = lrck_byte;

        if (!is_std && slot == 0 && lrck_byte) {
            for (uint16_t p = 1; p < sw; p++)
                dst[base + p] &= ~(1 << is->lrck_bit);
        }
    }
}

/*
 * Generic encoder for multi-protocol (I2S + SPDIF + ADAT combinations).
 */
static void IRAM_ATTR encode_frame_generic(struct parlio_audio_tx *h,
                                            uint8_t *dst,
                                            const int32_t *samples)
{
    const uint16_t tpf = h->ticks_per_frame;
    const uint8_t  pw  = h->parlio_width;

    uint8_t spdif_bits[16];
    uint8_t adat_bits[32];

    if (h->spdif) {
        const int32_t *sp = samples ? (samples + h->spdif_offset) : NULL;
        encode_spdif_raw(h->spdif, spdif_bits, sp ? sp[0] : 0, sp ? sp[1] : 0);
    }
    if (h->adat) {
        const int32_t *ap = samples ? (samples + h->adat_offset) : NULL;
        encode_adat_raw(h->adat, adat_bits, ap);
    }

    const int32_t *i2s_samp = (h->i2s && samples) ? samples + h->i2s_offset : NULL;
    const uint16_t *static_mask = h->tick_static_mask;
    const uint8_t  *slot_lut = h->tick_slot;
    const int8_t   *bidx_lut = h->tick_bit_idx;
    uint8_t i2s_ndl = 0, i2s_ns = 0, i2s_dsb = 0;
    if (h->i2s) {
        i2s_ndl = h->i2s->num_data_lines;
        i2s_ns  = h->i2s->num_slots;
        i2s_dsb = h->i2s->data_start_bit;
    }
    const uint8_t spdif_dbit = h->spdif ? h->spdif->data_bit : 0;
    const uint8_t adat_dbit  = h->adat  ? h->adat->data_bit  : 0;
    const uint16_t *spdif_ui_lut = h->tick_spdif_ui;
    const uint16_t *adat_bi_lut  = h->tick_adat_bi;
    const bool has_spdif = (h->spdif != NULL);
    const bool has_adat  = (h->adat  != NULL);

    for (uint16_t tick = 0; tick < tpf; tick++) {
        uint16_t word = static_mask[tick];

        int8_t bi = bidx_lut[tick];
        if (bi >= 0 && i2s_samp) {
            uint8_t sl = slot_lut[tick];
            uint32_t shift = 31 - (uint32_t)bi;
            for (uint8_t line = 0; line < i2s_ndl; line++)
                word |= (uint16_t)((i2s_samp[line * i2s_ns + sl] >> shift) & 1) << (i2s_dsb + line);
        }
        if (has_spdif) {
            uint16_t ui = spdif_ui_lut[tick];
            word |= (uint16_t)((spdif_bits[ui >> 3] >> (ui & 7)) & 1) << spdif_dbit;
        }
        if (has_adat) {
            uint16_t ai = adat_bi_lut[tick];
            word |= (uint16_t)((adat_bits[ai >> 3] >> (ai & 7)) & 1) << adat_dbit;
        }

        if (pw == 8) {
            dst[tick] = (uint8_t)word;
        } else {
            ((uint16_t *)dst)[tick] = word;
        }
    }
}

/* Dispatch to the appropriate encoder */
static inline void IRAM_ATTR encode_frame(struct parlio_audio_tx *h,
                                           uint8_t *dst,
                                           const int32_t *samples)
{
    if (h->i2s && !h->spdif && !h->adat) {
        encode_frame_i2s_only(h, dst, samples);
    } else {
        encode_frame_generic(h, dst, samples);
    }
}

/* ------------------------------------------------------------------ */
/*  ISR callback                                                       */
/* ------------------------------------------------------------------ */

static bool IRAM_ATTR on_tx_done(parlio_tx_unit_handle_t unit,
                                  const parlio_tx_done_event_data_t *edata,
                                  void *ctx)
{
    struct parlio_audio_tx *h = (struct parlio_audio_tx *)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(h->write_sem, &woken);
    return woken == pdTRUE;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

esp_err_t parlio_audio_tx_new(const parlio_audio_tx_config_t *config,
                               parlio_audio_tx_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(config->i2s || config->spdif || config->adat,
                        ESP_ERR_INVALID_ARG, TAG, "no protocol configured");

    /* --- Determine protocol clock rates --- */
    uint32_t fs = config->sample_rate;
    uint32_t adat_rate  = config->adat  ? 256 * fs : 0;
    uint32_t spdif_rate = config->spdif ? 128 * fs : 0;

    uint8_t i2s_num_slots = 0;
    uint32_t i2s_bclk_rate = 0;
    if (config->i2s) {
        i2s_num_slots = (uint8_t)config->i2s->mode;
        if (i2s_num_slots == 0) i2s_num_slots = 2;
        uint8_t sw = config->i2s->slot_width ? config->i2s->slot_width : 32;
        i2s_bclk_rate = (uint32_t)sw * i2s_num_slots * fs;
    }

    bool i2s_only = config->i2s && !config->spdif && !config->adat;
    bool multi_proto = !i2s_only && config->i2s;

    /* Master PARLIO clock = fastest protocol rate */
    uint32_t parlio_clock;
    if (i2s_only) {
        parlio_clock = i2s_bclk_rate; /* BCLK on clk_out, efficient mode */
    } else {
        parlio_clock = adat_rate;
        if (spdif_rate > parlio_clock) parlio_clock = spdif_rate;
        if (i2s_bclk_rate > parlio_clock) parlio_clock = i2s_bclk_rate;
    }
    ESP_RETURN_ON_FALSE(parlio_clock > 0, ESP_ERR_INVALID_ARG, TAG, "bad clock");

    /* Validate divisibility of each protocol rate into the master clock */
    if (config->spdif && parlio_clock % spdif_rate != 0) {
        ESP_LOGE(TAG, "PARLIO clock %"PRIu32" not divisible by S/PDIF rate %"PRIu32,
                 parlio_clock, spdif_rate);
        return ESP_ERR_INVALID_ARG;
    }
    if (config->adat && parlio_clock % adat_rate != 0) {
        ESP_LOGE(TAG, "PARLIO clock %"PRIu32" not divisible by ADAT rate %"PRIu32,
                 parlio_clock, adat_rate);
        return ESP_ERR_INVALID_ARG;
    }
    if (config->i2s && !i2s_only && parlio_clock % i2s_bclk_rate != 0) {
        ESP_LOGE(TAG, "PARLIO clock %"PRIu32" not divisible by I2S BCLK %"PRIu32". "
                 "Adjust slot_width so BCLK divides evenly into %"PRIu32,
                 parlio_clock, i2s_bclk_rate, parlio_clock);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t ticks_per_frame = parlio_clock / fs;

    /* --- Validate I2S config --- */
    if (config->i2s) {
        const parlio_audio_i2s_config_t *ic = config->i2s;
        ESP_RETURN_ON_FALSE(ic->num_data_lines >= 1 &&
                            ic->num_data_lines <= PARLIO_AUDIO_MAX_I2S_DATA_LINES,
                            ESP_ERR_INVALID_ARG, TAG, "I2S num_data_lines out of range");
        ESP_RETURN_ON_FALSE(ic->bits_per_sample == 16 || ic->bits_per_sample == 24 ||
                            ic->bits_per_sample == 32,
                            ESP_ERR_INVALID_ARG, TAG, "I2S bad bits_per_sample");
        uint8_t sw = ic->slot_width ? ic->slot_width : 32;
        ESP_RETURN_ON_FALSE(sw >= ic->bits_per_sample,
                            ESP_ERR_INVALID_ARG, TAG, "I2S slot_width < bits_per_sample");
    }

    /* --- Assign TXD bit positions --- */
    uint8_t bit_cursor = 0;
    uint8_t i2s_bclk_bit = 0, i2s_lrck_bit = 0, i2s_data_start = 0;
    uint8_t spdif_bit = 0, adat_bit = 0;

    if (config->i2s) {
        if (!i2s_only) {
            /* Multi-protocol: BCLK on a data line */
            i2s_bclk_bit = bit_cursor++;
        }
        i2s_lrck_bit = bit_cursor++;
        i2s_data_start = bit_cursor;
        bit_cursor += config->i2s->num_data_lines;
    }
    if (config->spdif) {
        spdif_bit = bit_cursor++;
    }
    if (config->adat) {
        adat_bit = bit_cursor++;
    }

    uint8_t pw = next_parlio_width(bit_cursor);
    ESP_RETURN_ON_FALSE(pw > 0 && pw <= 16, ESP_ERR_INVALID_ARG, TAG,
                        "too many data lines for PARLIO (need %u, max 16)", bit_cursor);

    /* --- MCLK multiple --- */
    uint16_t mclk_mult = config->mclk_multiple;
    if (mclk_mult == 0) {
        /* Auto: smallest multiple of ticks_per_frame that's >= ticks_per_frame */
        mclk_mult = ticks_per_frame;
        /* Ensure it's a supported I2S mclk_multiple value (multiple of 128 or 256) */
        if (mclk_mult < 256) mclk_mult = 256;
    }
    ESP_RETURN_ON_FALSE(mclk_mult >= ticks_per_frame,
                        ESP_ERR_INVALID_ARG, TAG,
                        "mclk_multiple (%u) too small for configured protocols "
                        "(need >= %u: %s%s%s)",
                        mclk_mult, ticks_per_frame,
                        config->i2s ? "I2S " : "",
                        config->spdif ? "SPDIF(128x) " : "",
                        config->adat ? "ADAT(256x)" : "");
    ESP_RETURN_ON_FALSE(mclk_mult % ticks_per_frame == 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "mclk_multiple (%u) not evenly divisible by ticks_per_frame (%u)",
                        mclk_mult, ticks_per_frame);

    /* Check APLL frequency limit (~50 MHz max) */
    uint32_t mclk_hz = (uint32_t)mclk_mult * fs;
    if (mclk_hz > 50000000) {
        ESP_LOGE(TAG, "MCLK = %"PRIu32" Hz exceeds APLL max (~50 MHz). "
                 "Reduce mclk_multiple (%u) or sample_rate (%"PRIu32")",
                 mclk_hz, mclk_mult, fs);
        return ESP_ERR_INVALID_ARG;
    }
    if (mclk_hz > 40000000) {
        ESP_LOGW(TAG, "MCLK = %"PRIu32" Hz is near APLL limit. "
                 "Some frequencies may not be achievable precisely", mclk_hz);
    }

    /* --- Allocate handle --- */
    struct parlio_audio_tx *h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "alloc handle");
    esp_err_t ret = ESP_OK;

    h->sample_rate   = fs;
    h->parlio_clock  = parlio_clock;
    h->ticks_per_frame = ticks_per_frame;
    h->parlio_width  = pw;
    h->mclk_multiple = mclk_mult;

    /* --- Populate protocol states --- */
    if (config->i2s) {
        h->i2s = calloc(1, sizeof(i2s_state_t));
        ESP_GOTO_ON_FALSE(h->i2s, ESP_ERR_NO_MEM, fail, TAG, "alloc i2s state");
        uint8_t sw = config->i2s->slot_width ? config->i2s->slot_width : 32;
        h->i2s->bclk_bit       = i2s_bclk_bit;
        h->i2s->lrck_bit       = i2s_lrck_bit;
        h->i2s->data_start_bit = i2s_data_start;
        h->i2s->num_data_lines = config->i2s->num_data_lines;
        h->i2s->slot_width     = sw;
        h->i2s->bits_per_sample = config->i2s->bits_per_sample;
        h->i2s->num_slots      = i2s_num_slots;
        h->i2s->mode           = config->i2s->mode ? config->i2s->mode : PARLIO_AUDIO_I2S_STANDARD;
        h->i2s->bclk_on_clk_out = i2s_only;
        h->i2s->ticks_per_bclk = i2s_only ? 1 : (parlio_clock / i2s_bclk_rate);
    }

    if (config->spdif) {
        h->spdif = calloc(1, sizeof(spdif_state_t));
        ESP_GOTO_ON_FALSE(h->spdif, ESP_ERR_NO_MEM, fail, TAG, "alloc spdif state");
        h->spdif->data_bit = spdif_bit;
        h->spdif->bits_per_sample = config->spdif->bits_per_sample ? config->spdif->bits_per_sample : 24;
        h->spdif->ticks_per_ui = parlio_clock / (128 * fs);
        memcpy(h->spdif->channel_status,
               config->spdif->consumer_format ? SPDIF_CS_CONSUMER : SPDIF_CS_PROFESSIONAL, 24);
    }

    if (config->adat) {
        h->adat = calloc(1, sizeof(adat_state_t));
        ESP_GOTO_ON_FALSE(h->adat, ESP_ERR_NO_MEM, fail, TAG, "alloc adat state");
        h->adat->data_bit = adat_bit;
        h->adat->ticks_per_bit = parlio_clock / (256 * fs);
    }

    /* --- Sample layout --- */
    size_t offset = 0;
    if (h->i2s) {
        h->i2s_offset = offset;
        offset += h->i2s->num_data_lines * h->i2s->num_slots;
    }
    if (h->spdif) {
        h->spdif_offset = offset;
        offset += 2;
    }
    if (h->adat) {
        h->adat_offset = offset;
        offset += 8;
    }
    h->samples_per_frame = offset;

    /* Build per-tick lookup tables (eliminates divisions in encode_frame) */
    build_tick_tables(h);
    ESP_GOTO_ON_FALSE(h->tick_static_mask && h->tick_bit_idx,
                      ESP_ERR_NO_MEM, fail, TAG, "alloc tick tables");

    /* Initialize per-line LUTs for I2S fast encoding */
    if (h->i2s) {
        ensure_line_luts(h->i2s->data_start_bit - 1 + h->i2s->num_data_lines);
    }

    /* --- DMA buffers (3 for loop mode rotation) --- */
    h->frames_per_buf = config->frames_per_buffer ? config->frames_per_buffer : 64;
    h->dma_buf_count  = 3; /* always 3 for loop DMA ping-pong-pong */

    /* With data_width < 8, PARLIO packs multiple words per byte (LSB order).
     * data_width=1: 8 ticks/byte, =2: 4 ticks/byte, =4: 2 ticks/byte,
     * =8: 1 tick/byte, =16: 2 bytes/tick */
    size_t ticks_per_byte = (pw < 8) ? (8 / pw) : 1;
    size_t bytes_per_tick = (pw > 8) ? 2 : 1;
    h->frame_buf_bytes = (pw <= 8)
        ? (ticks_per_frame + ticks_per_byte - 1) / ticks_per_byte
        : ticks_per_frame * bytes_per_tick;
    h->dma_buf_bytes   = h->frame_buf_bytes * h->frames_per_buf;

    h->dma_bufs = calloc(h->dma_buf_count, sizeof(uint8_t *));
    ESP_GOTO_ON_FALSE(h->dma_bufs, ESP_ERR_NO_MEM, fail, TAG, "alloc buf array");
    for (size_t i = 0; i < h->dma_buf_count; i++) {
        h->dma_bufs[i] = heap_caps_calloc(1, h->dma_buf_bytes,
                                           MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        ESP_GOTO_ON_FALSE(h->dma_bufs[i], ESP_ERR_NO_MEM, fail, TAG, "alloc DMA buf");
    }

    h->write_sem = xSemaphoreCreateCounting(h->dma_buf_count, h->dma_buf_count);
    ESP_GOTO_ON_FALSE(h->write_sem, ESP_ERR_NO_MEM, fail, TAG, "alloc semaphore");

    /* --- I2S peripheral: APLL clock + optional HW audio output --- */
    {
        const parlio_audio_i2s_hw_config_t *hw = config->i2s_hw;

        i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
        if (hw) {
            cc.dma_desc_num  = hw->dma_desc_num  ? hw->dma_desc_num  : 6;
            cc.dma_frame_num = hw->dma_frame_num ? hw->dma_frame_num : 240;
        } else {
            cc.dma_desc_num = 2;
            cc.dma_frame_num = 16;
        }
        ret = i2s_new_channel(&cc, &h->i2s_clk_chan, NULL);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "I2S channel alloc failed");

        if (hw && hw->total_slots > 2) {
            /* TDM mode: supports 4/8/16 slots and multiple data outputs */
            uint16_t nslots = hw->total_slots;
            i2s_tdm_slot_mask_t mask = (1 << nslots) - 1; /* enable all slots */

            i2s_tdm_config_t tc = {
                .clk_cfg = {
                    .sample_rate_hz = fs,
                    .clk_src = I2S_CLK_SRC_APLL,
                    .mclk_multiple = (i2s_mclk_multiple_t)mclk_mult,
                },
                .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
                    (i2s_data_bit_width_t)hw->bits_per_sample,
                    I2S_SLOT_MODE_STEREO, mask),
                .gpio_cfg = {
                    .mclk = config->mclk_gpio,
                    .bclk = hw->bclk_gpio,
                    .ws   = hw->ws_gpio,
                    .dout = hw->dout_gpio,
                    .din  = I2S_GPIO_UNUSED,
                },
            };
            if (hw->slot_width)
                tc.slot_cfg.slot_bit_width = (i2s_slot_bit_width_t)hw->slot_width;
            tc.slot_cfg.total_slot = nslots;

            ret = i2s_channel_init_tdm_mode(h->i2s_clk_chan, &tc);
            ESP_GOTO_ON_ERROR(ret, fail, TAG, "I2S TDM init failed");
            h->i2s_hw_enabled = true;

            /* Connect secondary data output if specified (I2S0 only) */
            /* The TDM driver doesn't expose dout2 in gpio_cfg, but the
             * secondary output shares the same data with slots split across
             * both pins. For now we log it; full multi-dout TDM requires
             * manual GPIO matrix routing after init. */
            if (hw->dout2_gpio >= 0) {
                ESP_LOGW(TAG, "I2S HW dout2 on GPIO %d -- multi-DOUT TDM requires "
                         "manual GPIO matrix setup for the secondary data pin",
                         hw->dout2_gpio);
            }

            ESP_LOGI(TAG, "  I2S HW: TDM%u, %u-bit, BCLK=%d WS=%d DOUT=%d",
                     nslots, hw->bits_per_sample, hw->bclk_gpio, hw->ws_gpio, hw->dout_gpio);
        } else if (hw) {
            /* Standard stereo mode */
            i2s_std_config_t sc = {
                .clk_cfg = { .sample_rate_hz = fs, .clk_src = I2S_CLK_SRC_APLL,
                             .mclk_multiple = (i2s_mclk_multiple_t)mclk_mult },
                .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                    (i2s_data_bit_width_t)hw->bits_per_sample, I2S_SLOT_MODE_STEREO),
                .gpio_cfg = {
                    .mclk = config->mclk_gpio,
                    .bclk = hw->bclk_gpio,
                    .ws   = hw->ws_gpio,
                    .dout = hw->dout_gpio,
                    .din  = I2S_GPIO_UNUSED,
                },
            };
            if (hw->slot_width)
                sc.slot_cfg.slot_bit_width = (i2s_slot_bit_width_t)hw->slot_width;
            ret = i2s_channel_init_std_mode(h->i2s_clk_chan, &sc);
            ESP_GOTO_ON_ERROR(ret, fail, TAG, "I2S STD init failed");
            h->i2s_hw_enabled = true;

            ESP_LOGI(TAG, "  I2S HW: stereo, %u-bit, BCLK=%d WS=%d DOUT=%d",
                     hw->bits_per_sample, hw->bclk_gpio, hw->ws_gpio, hw->dout_gpio);
        } else {
            /* Clock-only mode: minimal config, no audio output */
            i2s_std_config_t sc = {
                .clk_cfg = { .sample_rate_hz = fs, .clk_src = I2S_CLK_SRC_APLL,
                             .mclk_multiple = (i2s_mclk_multiple_t)mclk_mult },
                .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                                I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
                .gpio_cfg = { .mclk = config->mclk_gpio,
                              .bclk = I2S_GPIO_UNUSED, .ws = I2S_GPIO_UNUSED,
                              .dout = I2S_GPIO_UNUSED, .din = I2S_GPIO_UNUSED },
            };
            ret = i2s_channel_init_std_mode(h->i2s_clk_chan, &sc);
            ESP_GOTO_ON_ERROR(ret, fail, TAG, "I2S clock init failed");
        }

        ret = i2s_channel_enable(h->i2s_clk_chan);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "I2S enable failed");
    }

    /* --- PARLIO TX unit --- */
    {
        uint32_t mclk_freq = (uint32_t)mclk_mult * fs;
        parlio_tx_unit_config_t pc = {
            .clk_src = PARLIO_CLK_SRC_DEFAULT,
            .clk_in_gpio_num = config->mclk_gpio,
            .input_clk_src_freq_hz = mclk_freq,
            .output_clk_freq_hz = parlio_clock,
            .data_width = pw,
            .clk_out_gpio_num = config->clk_out_gpio,
            .valid_gpio_num = -1,
            .trans_queue_depth = h->dma_buf_count,
            .max_transfer_size = h->dma_buf_bytes,
            .sample_edge = PARLIO_SAMPLE_EDGE_POS,
            .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
            .flags = { .clk_gate_en = 0, .io_loop_back = 0 },
        };

        for (int i = 0; i < 16; i++) pc.data_gpio_nums[i] = -1;

        if (h->i2s) {
            if (i2s_only) {
                /* BCLK on clk_out (already set above), LRCK on TXD[0], data on TXD[1+] */
                pc.clk_out_gpio_num = config->i2s->bclk_gpio;
            } else {
                pc.data_gpio_nums[i2s_bclk_bit] = config->i2s->bclk_gpio;
            }
            pc.data_gpio_nums[i2s_lrck_bit] = config->i2s->lrck_gpio;
            for (uint8_t i = 0; i < config->i2s->num_data_lines; i++) {
                pc.data_gpio_nums[i2s_data_start + i] = config->i2s->data_gpios[i];
            }
        }
        if (h->spdif) {
            pc.data_gpio_nums[spdif_bit] = config->spdif->spdif_gpio;
        }
        if (h->adat) {
            pc.data_gpio_nums[adat_bit] = config->adat->adat_gpio;
        }

        ret = parlio_new_tx_unit(&pc, &h->parlio_unit);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "PARLIO create failed");

        parlio_tx_event_callbacks_t cbs = { .on_trans_done = on_tx_done };
        ret = parlio_tx_unit_register_event_callbacks(h->parlio_unit, &cbs, h);
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "PARLIO callbacks failed");
    }

    /* --- Log summary --- */
    ESP_LOGI(TAG, "parlio_clock=%"PRIu32" Hz, ticks/frame=%u, width=%u, MCLK=%"PRIu32" Hz",
             parlio_clock, ticks_per_frame, pw, (uint32_t)mclk_mult * fs);
    ESP_LOGI(TAG, "%u samples/frame, frame_buf=%u B, dma=%u B x %u",
             (unsigned)h->samples_per_frame, (unsigned)h->frame_buf_bytes,
             (unsigned)h->dma_buf_bytes, (unsigned)h->dma_buf_count);
    if (h->i2s)   ESP_LOGI(TAG, "  I2S: %u lines x %u slots, BCLK=%s, ticks/bclk=%u",
                            h->i2s->num_data_lines, h->i2s->num_slots,
                            h->i2s->bclk_on_clk_out ? "clk_out" : "TXD",
                            h->i2s->ticks_per_bclk);
    if (h->spdif) ESP_LOGI(TAG, "  SPDIF: TXD[%u], ticks/ui=%u", h->spdif->data_bit, h->spdif->ticks_per_ui);
    if (h->adat)  ESP_LOGI(TAG, "  ADAT: TXD[%u], ticks/bit=%u", h->adat->data_bit, h->adat->ticks_per_bit);

    *ret_handle = h;
    return ESP_OK;

fail:
    parlio_audio_tx_delete(h);
    return ESP_FAIL;
}

esp_err_t parlio_audio_tx_enable(parlio_audio_tx_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle && !handle->enabled, ESP_ERR_INVALID_STATE, TAG, "bad state");
    ESP_RETURN_ON_ERROR(parlio_tx_unit_enable(handle->parlio_unit), TAG, "enable failed");

    handle->enabled = true;
    handle->loop_mode = true;

    /* Reset encoder states */
    if (handle->spdif) { handle->spdif->frame_in_block = 0; handle->spdif->bmc_state = 0; }
    if (handle->adat)  { handle->adat->nrzi_state = 0; }

    /* Pre-fill buf[0] with silence and start loop DMA */
    for (size_t f = 0; f < handle->frames_per_buf; f++) {
        uint8_t *dst = handle->dma_bufs[0] + f * handle->frame_buf_bytes;
        encode_frame(handle, dst, NULL);
    }
    parlio_transmit_config_t tc = {
        .idle_value = 0,
        .flags.loop_transmission = 1,
    };
    ESP_RETURN_ON_ERROR(
        parlio_tx_unit_transmit(handle->parlio_unit, handle->dma_bufs[0],
                                handle->dma_buf_bytes * 8, &tc),
        TAG, "initial loop transmit failed");

    handle->write_buf_idx = 1;
    handle->write_frame_pos = 0;
    handle->start_time_us = esp_timer_get_time();
    handle->total_submitted = handle->frames_per_buf;

    ESP_LOGD(TAG, "enabled (loop DMA), Fs=%"PRIu32, handle->sample_rate);
    return ESP_OK;
}

esp_err_t parlio_audio_tx_disable(parlio_audio_tx_handle_t handle)
{
    if (!handle || !handle->enabled) return ESP_OK;
    if (!handle->loop_mode)
        parlio_tx_unit_wait_all_done(handle->parlio_unit, 500);
    parlio_tx_unit_disable(handle->parlio_unit);
    handle->enabled = false;
    return ESP_OK;
}

esp_err_t parlio_audio_tx_write(parlio_audio_tx_handle_t handle,
                                 const int32_t *samples,
                                 size_t num_frames,
                                 size_t *frames_written,
                                 uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && samples && handle->enabled,
                        ESP_ERR_INVALID_ARG, TAG, "bad arg/state");

    const size_t spf = handle->samples_per_frame;
    size_t written = 0;

    while (written < num_frames) {
        uint8_t *dst = handle->dma_bufs[handle->write_buf_idx]
                     + handle->write_frame_pos * handle->frame_buf_bytes;
        encode_frame(handle, dst, &samples[written * spf]);
        handle->write_frame_pos++;
        written++;

        /* Buffer full: pace and submit */
        if (handle->write_frame_pos >= handle->frames_per_buf) {
            if (handle->loop_mode) {
                /* Absolute frame-count timing for loop mode */
                handle->total_submitted += handle->frames_per_buf;
                int64_t target_us = handle->start_time_us
                    + (int64_t)(handle->total_submitted * 1000000ULL / handle->sample_rate);
                int64_t now = esp_timer_get_time();
                if (target_us > now)
                    esp_rom_delay_us((uint32_t)(target_us - now));
            } else {
                /* Semaphore-based flow control for one-shot mode */
                if (xSemaphoreTake(handle->write_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
                    if (frames_written) *frames_written = written;
                    return ESP_ERR_TIMEOUT;
                }
            }

            parlio_transmit_config_t tc = {
                .idle_value = 0,
                .flags.loop_transmission = handle->loop_mode ? 1 : 0,
            };
            esp_err_t ret = parlio_tx_unit_transmit(
                handle->parlio_unit, handle->dma_bufs[handle->write_buf_idx],
                handle->dma_buf_bytes * 8, &tc);
            ESP_RETURN_ON_ERROR(ret, TAG, "transmit failed");

            handle->write_buf_idx = (handle->write_buf_idx + 1) % handle->dma_buf_count;
            handle->write_frame_pos = 0;
        }
    }

    if (frames_written) *frames_written = written;
    return ESP_OK;
}

esp_err_t parlio_audio_tx_delete(parlio_audio_tx_handle_t handle)
{
    if (!handle) return ESP_OK;
    if (handle->enabled) parlio_audio_tx_disable(handle);
    if (handle->parlio_unit) parlio_del_tx_unit(handle->parlio_unit);
    if (handle->i2s_clk_chan) {
        i2s_channel_disable(handle->i2s_clk_chan);
        i2s_del_channel(handle->i2s_clk_chan);
    }
    if (handle->dma_bufs) {
        for (size_t i = 0; i < handle->dma_buf_count; i++) free(handle->dma_bufs[i]);
        free(handle->dma_bufs);
    }
    if (handle->write_sem) vSemaphoreDelete(handle->write_sem);
    free(handle->tick_static_mask);
    free(handle->tick_slot);
    free(handle->tick_bit_idx);
    free(handle->tick_spdif_ui);
    free(handle->tick_adat_bi);
    free(handle->i2s);
    free(handle->spdif);
    free(handle->adat);
    free(handle);
    return ESP_OK;
}

size_t parlio_audio_tx_get_frame_size(parlio_audio_tx_handle_t handle)
{
    return handle ? handle->samples_per_frame : 0;
}

uint32_t parlio_audio_tx_get_parlio_clock(parlio_audio_tx_handle_t handle)
{
    return handle ? handle->parlio_clock : 0;
}

i2s_chan_handle_t parlio_audio_tx_get_i2s_hw_handle(parlio_audio_tx_handle_t handle)
{
    if (handle && handle->i2s_hw_enabled) return handle->i2s_clk_chan;
    return NULL;
}
