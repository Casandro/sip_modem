#ifndef FSK_H
#define FSK_H

#include <stdint.h>

/* ── Combined V.21 / V.23 / Bell 103 FSK modem core ───────────────────
 *
 * One parameterized modem replaces the former v21.c and v23.c. A
 * fsk_profile_t carries everything that differs between the standards
 * (tone frequencies, baud, Goertzel window, carrier threshold); the
 * modulator, demodulator, and framers are profile-independent.
 *
 *   V.21    — symmetric 300 bps full-duplex, two channel pairs:
 *               originate TX 980/1180   RX 1650/1850
 *               answer    TX 1650/1850  RX 980/1180
 *   Bell 103 — symmetric 300 bps full-duplex, the US/Bell counterpart of
 *              V.21 (same structure, different tones, mark = higher freq):
 *               originate TX 1270/1070  RX 2225/2025  (mark/space)
 *               answer    TX 2225/2025  RX 1270/1070
 *   V.23    — asymmetric: TX 1300/2100 @1200 forward, RX 390/450 @75 back.
 *             Modelled as the answer/host side only.
 */

/* ── Framing ──────────────────────────────────────────────────────────
 * sync=1 streams raw 8-bit LSB-first bytes with no start/stop/parity;
 * the other fields are then unused placeholders. */
typedef struct {
    int  sync;        /* 0 = async (start/data/[parity]/stop), 1 = sync streaming */
    int  data_bits;   /* 5..8 — async only */
    char parity;      /* 'N', 'E', or 'O' — async only */
    int  stop_bits;   /* 1 or 2 — async only */
} fsk_framing_t;

/* Parse "8N1", "7E1", … or "sync"/"SYNC" into f. Returns 0 on success,
 * -1 on error. In sync mode data_bits/parity/stop_bits are filled with
 * placeholders (8/N/1) but unused by the modulator/demodulator. */
int fsk_parse_framing(const char *s, fsk_framing_t *f);

/* ── Profile ──────────────────────────────────────────────────────────
 * The complete set of parameters that distinguish the two standards (and
 * the two V.21 sides). Resolved once via fsk_profile_init(); the rest of
 * the core reads only the derived increments/coefficients, never raw Hz. */
typedef struct {
    double tx_mark_hz, tx_space_hz;  int tx_baud;
    double rx_mark_hz, rx_space_hz;  int rx_baud, rx_win;
    double carrier_threshold;
} fsk_profile_t;

typedef enum { FSK_V21 = 0, FSK_V23 = 1, FSK_BELL103 = 2 } fsk_mode_t;

/* Fill p for the given mode. `originate` selects the side for the
 * symmetric standards (V.21 and Bell 103): 1 = originate, 0 = answer.
 * It is ignored for V.23 (host/answer side only). */
void fsk_profile_init(fsk_profile_t *p, fsk_mode_t mode, int originate);

/* ── Process-wide one-time init for the shared sine LUT (idempotent). ── */
void fsk_init(void);

/* ── Modulator ────────────────────────────────────────────────────────
 * Sizes the RX ring; must be >= every profile's rx_win (V.23 = 160). */
#define FSK_RX_WIN_MAX 160

typedef struct {
    /* Phase-continuous carrier, Q32 cycles. inc_cur is the per-sample
     * phase increment for whichever tone (mark/space) the current bit
     * selected. inc_mark/inc_space are precomputed once at init. */
    uint32_t phase;
    uint32_t inc_cur;
    uint32_t inc_mark;
    uint32_t inc_space;

    /* Bit clock: Q16 sample counter that overflows once per bit-period. */
    uint32_t bit_phase;
    uint32_t bit_inc;     /* (tx_baud << 16) / fs */

    /* Async framer: IDLE -> START -> D0..Dn-1 -> [PARITY] -> STOP.
     * Sync framer: IDLE -> SYNC_DATA (bit 0..7) -> SYNC_DATA / IDLE. */
    int frame_state;      /* see FSK_TX_* in fsk.c */
    int bit_index;
    int stop_index;
    uint8_t cur_byte;
    int     parity_bit;

    fsk_framing_t fr;
    int16_t amplitude;

    /* If non-zero, fsk_tx_sample() emits a continuous space tone — a
     * serial "break" — and freezes the bit clock. Phase stays continuous
     * with the surrounding mark, so toggling it causes no audible click.
     * Intended for V.23's forward channel (see modem_fsk.c -B). */
    int     break_mode;
} fsk_tx_t;

void    fsk_tx_init(fsk_tx_t *t, const fsk_profile_t *p,
                    const fsk_framing_t *fr, int16_t amplitude);
void    fsk_tx_set_break(fsk_tx_t *t, int on);

/* Produce one sample at 8 kHz. Pulls one byte from buf (length-tracked
 * via *len) whenever a new frame is needed; otherwise stays idle (mark). */
int16_t fsk_tx_sample(fsk_tx_t *t, uint8_t *buf, int *len);

/* ── Demodulator ──────────────────────────────────────────────────────
 * Single-bin Goertzel over the last rx_win samples. The window equals
 * 1.5 bit periods (V.21) / 20 ms (V.23); bin width = fs/rx_win matches
 * the mark/space spacing, putting each tone on the other's first null. */
typedef struct {
    int16_t ring[FSK_RX_WIN_MAX];
    int     rx_win;        /* active window length (<= FSK_RX_WIN_MAX) */
    int     ring_pos;
    int     warm;          /* samples seen so far, capped at rx_win */

    double  coeff_mark;    /* 2*cos(2π f/fs) */
    double  coeff_space;
    double  mag2_mark;
    double  mag2_space;

    int     last_bit;      /* hysteretic current bit: 0 = space, 1 = mark */

    int     frame_state;   /* see FSK_RX_* in fsk.c */
    uint32_t bit_phase;    /* Q16 */
    uint32_t bit_inc;      /* (rx_baud << 16) / fs */
    int     bit_index;
    int     stop_index;
    uint8_t shift_reg;
    int     parity_acc;

    fsk_framing_t fr;

    /* Carrier detection: carrier_on flips after on_hold consecutive
     * samples above threshold, and back off after off_hold below it.
     * Defaults set in fsk_rx_init(): ~100 ms on, ~300 ms off. */
    double  carrier_threshold;
    int     carrier_on_hold;
    int     carrier_off_hold;
    int     carrier_streak;
    int     silence_streak;
    int     carrier_on;
} fsk_rx_t;

void fsk_rx_init(fsk_rx_t *r, const fsk_profile_t *p, const fsk_framing_t *fr);

/* Set the energy threshold (mag²_mark + mag²_space) above which carrier
 * is counted. Pass 0 to keep the current value. */
void fsk_rx_set_carrier_threshold(fsk_rx_t *r, double thresh);

/* 1 if a carrier is currently asserted (with hysteresis), else 0. */
int  fsk_rx_carrier(const fsk_rx_t *r);

/* Push one 8 kHz int16 sample. Returns:
 *   0..255  — a complete byte was just decoded
 *   -1      — no byte ready yet
 *   -2      — async-only: frame/parity error (framer auto-recovers).
 */
int  fsk_rx_sample(fsk_rx_t *r, int16_t sample);

#endif /* FSK_H */
