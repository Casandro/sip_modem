#define _POSIX_C_SOURCE 200809L
#include "fsk.h"
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Constants ────────────────────────────────────────────────────── */

#define FSK_FS              8000

/* V.21 channel pairs (ITU-T V.21, FSK, 300 bps full-duplex). */
#define V21_BAUD             300
#define V21_ORIG_MARK_HZ     980
#define V21_ORIG_SPACE_HZ   1180
#define V21_ANS_MARK_HZ     1650
#define V21_ANS_SPACE_HZ    1850
#define V21_RX_WIN            40
#define V21_THRESHOLD       6.25e6

/* Bell 103 channel pairs (300 bps full-duplex; mark = the higher tone).
 * Same symmetric originate/answer structure as V.21, different tones. */
#define B103_BAUD            300
#define B103_ORIG_MARK_HZ   1270
#define B103_ORIG_SPACE_HZ  1070
#define B103_ANS_MARK_HZ    2225
#define B103_ANS_SPACE_HZ   2025
#define B103_RX_WIN           40
#define B103_THRESHOLD      6.25e6

/* V.23 forward 1200 bps / back-channel 75 bps (host/answer side). */
#define V23_TX_MARK_HZ      1300
#define V23_TX_SPACE_HZ     2100
#define V23_RX_MARK_HZ       390
#define V23_RX_SPACE_HZ      450
#define V23_TX_BAUD         1200
#define V23_RX_BAUD           75
#define V23_RX_WIN           160
#define V23_THRESHOLD       1.0e8

#define LUT_BITS   12
#define LUT_SIZE   (1u << LUT_BITS)     /* 4096 */
#define LUT_MASK   (LUT_SIZE - 1u)

/* Shared 16-bit sine LUT, full-scale ±32767. */
static int16_t g_lut[LUT_SIZE];
static int     g_lut_ready = 0;

void fsk_init(void) {
    if (g_lut_ready) return;
    for (unsigned i = 0; i < LUT_SIZE; i++) {
        double th = 2.0 * M_PI * (double)i / (double)LUT_SIZE;
        g_lut[i] = (int16_t)lrint(sin(th) * 32767.0);
    }
    g_lut_ready = 1;
}

static uint32_t phase_inc_for(double hz) {
    return (uint32_t)((hz * 4294967296.0 / (double)FSK_FS) + 0.5);
}

/* Advance a Q16 bit clock by inc. Returns 1 when it rolls over a full
 * bit period (and wraps *phase), 0 otherwise. */
static inline int bit_tick(uint32_t *phase, uint32_t inc) {
    uint32_t bp = *phase + inc;
    if (bp >= 0x10000u) { *phase = bp - 0x10000u; return 1; }
    *phase = bp;
    return 0;
}

/* ── Profile ──────────────────────────────────────────────────────── */

void fsk_profile_init(fsk_profile_t *p, fsk_mode_t mode, int originate) {
    if (mode == FSK_V23) {
        /* Host/answer side: transmit the 1200 forward channel, receive
         * the 75 bps back channel. (`originate` is intentionally ignored.) */
        p->tx_mark_hz = V23_TX_MARK_HZ; p->tx_space_hz = V23_TX_SPACE_HZ;
        p->tx_baud    = V23_TX_BAUD;
        p->rx_mark_hz = V23_RX_MARK_HZ; p->rx_space_hz = V23_RX_SPACE_HZ;
        p->rx_baud    = V23_RX_BAUD;    p->rx_win      = V23_RX_WIN;
        p->carrier_threshold = V23_THRESHOLD;
        return;
    }
    /* Symmetric 300 bps full-duplex (V.21 and Bell 103): the two sides swap
     * mark/space pairs so the channels don't interfere. The modulator emits
     * the local side's pair; the demodulator listens on the opposite pair.
     * Only the tone table and baud/window/threshold differ between the two. */
    double orig_mark, orig_space, ans_mark, ans_space;
    int    baud, rx_win;
    double threshold;
    if (mode == FSK_BELL103) {
        orig_mark = B103_ORIG_MARK_HZ; orig_space = B103_ORIG_SPACE_HZ;
        ans_mark  = B103_ANS_MARK_HZ;  ans_space  = B103_ANS_SPACE_HZ;
        baud = B103_BAUD; rx_win = B103_RX_WIN; threshold = B103_THRESHOLD;
    } else { /* FSK_V21 */
        orig_mark = V21_ORIG_MARK_HZ; orig_space = V21_ORIG_SPACE_HZ;
        ans_mark  = V21_ANS_MARK_HZ;  ans_space  = V21_ANS_SPACE_HZ;
        baud = V21_BAUD; rx_win = V21_RX_WIN; threshold = V21_THRESHOLD;
    }
    double local_mark  = originate ? orig_mark  : ans_mark;
    double local_space = originate ? orig_space : ans_space;
    double far_mark    = originate ? ans_mark   : orig_mark;
    double far_space   = originate ? ans_space  : orig_space;
    p->tx_mark_hz = local_mark; p->tx_space_hz = local_space; p->tx_baud = baud;
    p->rx_mark_hz = far_mark;   p->rx_space_hz = far_space;   p->rx_baud = baud;
    p->rx_win     = rx_win;
    p->carrier_threshold = threshold;
}

/* ── Framing parsing ──────────────────────────────────────────────── */

int fsk_parse_framing(const char *s, fsk_framing_t *f) {
    if (!s) return -1;
    if (strcasecmp(s, "sync") == 0) {
        f->sync      = 1;
        f->data_bits = 8;   /* placeholders, unused in sync mode */
        f->parity    = 'N';
        f->stop_bits = 1;
        return 0;
    }
    if (strlen(s) != 3) return -1;
    if (!isdigit((unsigned char)s[0]) || !isdigit((unsigned char)s[2])) return -1;
    int db = s[0] - '0';
    if (db < 5 || db > 8) return -1;
    char par = (char)toupper((unsigned char)s[1]);
    if (par != 'N' && par != 'E' && par != 'O') return -1;
    int sb = s[2] - '0';
    if (sb < 1 || sb > 2) return -1;
    f->sync      = 0;
    f->data_bits = db;
    f->parity    = par;
    f->stop_bits = sb;
    return 0;
}

static int compute_parity_bit(uint8_t byte, int data_bits, char par) {
    if (par == 'N') return 0;
    int ones = 0;
    for (int i = 0; i < data_bits; i++)
        if (byte & (1u << i)) ones++;
    int even = (ones & 1) == 0;
    /* Even parity: total (data + parity) ones even. Odd parity: total odd. */
    if (par == 'E') return even ? 0 : 1;
    return even ? 1 : 0;
}

/* ── Modulator ────────────────────────────────────────────────────── */

enum {
    FSK_TX_IDLE = 0,
    FSK_TX_START,
    FSK_TX_DATA,
    FSK_TX_PARITY,
    FSK_TX_STOP,
    FSK_TX_SYNC_DATA     /* sync mode: streaming bits 0..7 of cur_byte */
};

void fsk_tx_init(fsk_tx_t *t, const fsk_profile_t *p,
                 const fsk_framing_t *fr, int16_t amplitude) {
    fsk_init();
    memset(t, 0, sizeof(*t));
    t->inc_mark    = phase_inc_for(p->tx_mark_hz);
    t->inc_space   = phase_inc_for(p->tx_space_hz);
    t->inc_cur     = t->inc_mark;                  /* start on mark = idle */
    t->bit_inc     = ((uint32_t)p->tx_baud << 16) / FSK_FS;
    t->frame_state = FSK_TX_IDLE;
    t->fr          = *fr;
    t->amplitude   = amplitude;
}

void fsk_tx_set_break(fsk_tx_t *t, int on) {
    t->break_mode = on ? 1 : 0;
}

/* Look up the next bit (0=space, 1=mark) and set inc_cur accordingly. */
static void tx_apply_bit(fsk_tx_t *t, int bit) {
    t->inc_cur = bit ? t->inc_mark : t->inc_space;
}

/* Pull one byte off buf (FIFO). Returns 1 if pulled, 0 if empty. */
static int tx_pull_byte(fsk_tx_t *t, uint8_t *buf, int *len) {
    if (*len <= 0) return 0;
    t->cur_byte = buf[0];
    memmove(buf, buf + 1, (size_t)(*len - 1));
    (*len)--;
    return 1;
}

/* Begin a new async frame if a byte is available, else stay idle (mark). */
static void start_frame(fsk_tx_t *t, uint8_t *buf, int *len) {
    if (tx_pull_byte(t, buf, len)) {
        t->parity_bit  = compute_parity_bit(t->cur_byte, t->fr.data_bits, t->fr.parity);
        t->frame_state = FSK_TX_START;
        t->bit_index   = 0;
        tx_apply_bit(t, 0);                        /* start bit = space */
    } else {
        tx_apply_bit(t, 1);                        /* idle = mark */
    }
}

/* Advance the framer by one bit. Called when the bit clock rolls over. */
static void tx_advance(fsk_tx_t *t, uint8_t *buf, int *len) {
    /* Sync mode: stream the 8 bits of cur_byte LSB-first, then fetch the
     * next byte (or idle on mark). No start/stop/parity bits. */
    if (t->fr.sync) {
        switch (t->frame_state) {
        case FSK_TX_SYNC_DATA:
            t->bit_index++;
            if (t->bit_index < 8) {
                tx_apply_bit(t, (t->cur_byte >> t->bit_index) & 1);
                return;
            }
            /* fall through to fetch next byte */
            break;
        default:
            break;  /* IDLE (or recovery): fetch a byte below */
        }
        if (tx_pull_byte(t, buf, len)) {
            t->frame_state = FSK_TX_SYNC_DATA;
            t->bit_index   = 0;
            tx_apply_bit(t, t->cur_byte & 1);
        } else {
            t->frame_state = FSK_TX_IDLE;
            tx_apply_bit(t, 1);                    /* idle = continuous mark */
        }
        return;
    }

    /* Async framer (start / data / [parity] / stop). */
    switch (t->frame_state) {
    case FSK_TX_IDLE:
        start_frame(t, buf, len);
        break;
    case FSK_TX_START:
        t->frame_state = FSK_TX_DATA;
        t->bit_index   = 0;
        tx_apply_bit(t, (t->cur_byte >> 0) & 1);
        break;
    case FSK_TX_DATA:
        t->bit_index++;
        if (t->bit_index < t->fr.data_bits) {
            tx_apply_bit(t, (t->cur_byte >> t->bit_index) & 1);
        } else if (t->fr.parity != 'N') {
            t->frame_state = FSK_TX_PARITY;
            tx_apply_bit(t, t->parity_bit);
        } else {
            t->frame_state = FSK_TX_STOP;
            t->stop_index = 0;
            tx_apply_bit(t, 1);                    /* stop bit = mark */
        }
        break;
    case FSK_TX_PARITY:
        t->frame_state = FSK_TX_STOP;
        t->stop_index = 0;
        tx_apply_bit(t, 1);
        break;
    case FSK_TX_STOP:
        t->stop_index++;
        if (t->stop_index < t->fr.stop_bits) {
            tx_apply_bit(t, 1);
        } else {
            /* Start the next byte immediately (no extra idle bit between
             * back-to-back bytes), or fall back to idle mark. */
            t->frame_state = FSK_TX_IDLE;
            start_frame(t, buf, len);
        }
        break;
    }
}

int16_t fsk_tx_sample(fsk_tx_t *t, uint8_t *buf, int *len) {
    /* Break override: force the space tone and freeze the bit clock so
     * the framer stays put until break is released. Phase keeps advancing,
     * so we transition cleanly into/out of break. */
    if (t->break_mode) t->inc_cur = t->inc_space;

    /* Output sample using current carrier choice. Linear-interpolated LUT:
     * top LUT_BITS index, low (32-LUT_BITS) fraction. */
    uint32_t idx_hi  = t->phase >> (32 - LUT_BITS);
    uint32_t idx_lo  = (t->phase << LUT_BITS) >> 16;      /* Q16 frac */
    int32_t  a = g_lut[idx_hi];
    int32_t  b = g_lut[(idx_hi + 1) & LUT_MASK];
    int32_t  v = a + (((b - a) * (int32_t)idx_lo) >> 16);
    int32_t  out = (v * t->amplitude) >> 15;
    if (out > 32767) out = 32767;
    if (out < -32768) out = -32768;

    /* Advance phase. Bit clock is frozen while we're in break. */
    t->phase += t->inc_cur;
    if (!t->break_mode && bit_tick(&t->bit_phase, t->bit_inc))
        tx_advance(t, buf, len);
    return (int16_t)out;
}

/* ── Demodulator ──────────────────────────────────────────────────── */

enum {
    FSK_RX_IDLE_MARK = 0,
    FSK_RX_START_VALIDATE,
    FSK_RX_DATA,
    FSK_RX_PARITY,
    FSK_RX_STOP,
    FSK_RX_SYNC_WAIT_CD,     /* sync mode: waiting for carrier-on */
    FSK_RX_SYNC_DATA         /* sync mode: clocking 8 bits per byte */
};

void fsk_rx_init(fsk_rx_t *r, const fsk_profile_t *p, const fsk_framing_t *fr) {
    memset(r, 0, sizeof(*r));
    r->rx_win      = p->rx_win;
    if (r->rx_win > FSK_RX_WIN_MAX) r->rx_win = FSK_RX_WIN_MAX;  /* ring bound */
    r->coeff_mark  = 2.0 * cos(2.0 * M_PI * p->rx_mark_hz  / (double)FSK_FS);
    r->coeff_space = 2.0 * cos(2.0 * M_PI * p->rx_space_hz / (double)FSK_FS);
    r->last_bit    = 1;                            /* assume line is mark */
    r->bit_inc     = ((uint32_t)p->rx_baud << 16) / FSK_FS;
    r->frame_state = fr->sync ? FSK_RX_SYNC_WAIT_CD : FSK_RX_IDLE_MARK;
    r->fr          = *fr;

    r->carrier_threshold = p->carrier_threshold;
    r->carrier_on_hold   = 800;   /* 100 ms @ 8 kHz */
    r->carrier_off_hold  = 2400;  /* 300 ms */
}

void fsk_rx_set_carrier_threshold(fsk_rx_t *r, double thresh) {
    if (thresh > 0.0) r->carrier_threshold = thresh;
}

int fsk_rx_carrier(const fsk_rx_t *r) {
    return r->carrier_on;
}

/* Single-bin Goertzel over the warm portion of r->ring (oldest-to-newest)
 * for the supplied 2*cos coefficient. Returns squared magnitude. */
static double goertzel_mag2(const fsk_rx_t *r, double coeff) {
    double s_prev = 0.0, s_prev2 = 0.0;
    int n = r->warm;
    int idx = (n < r->rx_win) ? 0 : r->ring_pos;   /* oldest sample */
    for (int i = 0; i < n; i++) {
        double x = (double)r->ring[idx];
        double s = x + coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev  = s;
        idx++;
        if (idx == r->rx_win) idx = 0;
    }
    return s_prev * s_prev + s_prev2 * s_prev2 - coeff * s_prev * s_prev2;
}

/* Sample the current "bit" at this instant — 1 = mark, 0 = space. Also
 * updates the carrier-detect hysteresis from the two FSK-bin energies. */
static int rx_sense_bit(fsk_rx_t *r) {
    r->mag2_mark  = goertzel_mag2(r, r->coeff_mark);
    r->mag2_space = goertzel_mag2(r, r->coeff_space);
    /* Hysteresis: require dominant > 1.3× other to switch. */
    if (r->last_bit) {
        if (r->mag2_space > r->mag2_mark * 1.3) r->last_bit = 0;
    } else {
        if (r->mag2_mark > r->mag2_space * 1.3) r->last_bit = 1;
    }
    double total = r->mag2_mark + r->mag2_space;
    if (total >= r->carrier_threshold) {
        r->silence_streak = 0;
        if (r->carrier_streak < r->carrier_on_hold)
            r->carrier_streak++;
        if (!r->carrier_on && r->carrier_streak >= r->carrier_on_hold)
            r->carrier_on = 1;
    } else {
        r->carrier_streak = 0;
        if (r->silence_streak < r->carrier_off_hold)
            r->silence_streak++;
        if (r->carrier_on && r->silence_streak >= r->carrier_off_hold)
            r->carrier_on = 0;
    }
    return r->last_bit;
}

int fsk_rx_sample(fsk_rx_t *r, int16_t sample) {
    /* Push into ring buffer. */
    r->ring[r->ring_pos] = sample;
    r->ring_pos = (r->ring_pos + 1) % r->rx_win;
    if (r->warm < r->rx_win) r->warm++;

    /* Skip until the window is warm so the initial transient doesn't
     * trigger spurious start bits. */
    if (r->warm < r->rx_win) return -1;

    int bit_now = rx_sense_bit(r);
    int rv = -1;

    switch (r->frame_state) {
    case FSK_RX_IDLE_MARK:
        if (!bit_now) {
            /* Falling edge mark->space: begin start-bit validation. Seed
             * the clock at ½ a bit so the next rollover lands mid-bit. */
            r->frame_state = FSK_RX_START_VALIDATE;
            r->bit_phase   = 0x8000u;
        }
        break;
    case FSK_RX_START_VALIDATE:
        if (bit_tick(&r->bit_phase, r->bit_inc)) {
            if (bit_now) {
                /* Glitch: line went back to mark too soon. Abort. */
                r->frame_state = FSK_RX_IDLE_MARK;
            } else {
                r->frame_state = FSK_RX_DATA;
                r->bit_index   = 0;
                r->shift_reg   = 0;
                r->parity_acc  = 0;
            }
        }
        break;
    case FSK_RX_DATA:
        if (bit_tick(&r->bit_phase, r->bit_inc)) {
            /* LSB-first: a space (0) on the line is a 0 bit. */
            int b = bit_now ? 1 : 0;
            r->shift_reg |= (uint8_t)(b << r->bit_index);
            r->parity_acc ^= b;
            r->bit_index++;
            if (r->bit_index >= r->fr.data_bits) {
                if (r->fr.parity != 'N') {
                    r->frame_state = FSK_RX_PARITY;
                } else {
                    r->frame_state = FSK_RX_STOP;
                    r->stop_index  = 0;
                }
            }
        }
        break;
    case FSK_RX_PARITY:
        if (bit_tick(&r->bit_phase, r->bit_inc)) {
            int b = bit_now ? 1 : 0;
            int expect = (r->fr.parity == 'E') ? r->parity_acc
                                               : (r->parity_acc ^ 1);
            if (b != expect) {
                r->frame_state = FSK_RX_IDLE_MARK;
                rv = -2;                            /* parity error */
                break;
            }
            r->frame_state = FSK_RX_STOP;
            r->stop_index  = 0;
        }
        break;
    case FSK_RX_STOP:
        if (bit_tick(&r->bit_phase, r->bit_inc)) {
            if (!bit_now) {
                /* Framing error: expected mark. */
                r->frame_state = FSK_RX_IDLE_MARK;
                rv = -2;
                break;
            }
            r->stop_index++;
            if (r->stop_index >= r->fr.stop_bits) {
                rv = r->shift_reg;
                r->frame_state = FSK_RX_IDLE_MARK;
            }
        }
        break;

    /* Sync mode: there's no start bit to align on, so we use the
     * carrier-detect transition as the sync point. From CD-rising onward
     * we clock 8 bits per byte continuously; if CD drops we wait and
     * re-sync on the next CD-rising edge. Idle = continuous mark =>
     * 0xFF bytes emitted at baud/8 when the remote isn't sending data. */
    case FSK_RX_SYNC_WAIT_CD:
        if (r->carrier_on) {
            r->frame_state = FSK_RX_SYNC_DATA;
            r->bit_phase   = 0x8000u;              /* sample at mid-bit */
            r->bit_index   = 0;
            r->shift_reg   = 0;
        }
        break;
    case FSK_RX_SYNC_DATA:
        if (!r->carrier_on) {
            r->frame_state = FSK_RX_SYNC_WAIT_CD;
            break;
        }
        if (bit_tick(&r->bit_phase, r->bit_inc)) {
            int b = bit_now ? 1 : 0;
            r->shift_reg |= (uint8_t)(b << r->bit_index);
            r->bit_index++;
            if (r->bit_index >= 8) {
                rv = r->shift_reg;
                r->bit_index = 0;
                r->shift_reg = 0;
            }
        }
        break;
    }
    return rv;
}
