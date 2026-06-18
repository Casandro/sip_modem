#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "v22bis.h"
#include "v22bis_rrc.h"
#include <string.h>
#include <math.h>

/* ── V.22bis transmitter ───────────────────────────────────────────────
 * Scrambler -> differential 16-QAM mapper -> root-raised-cosine polyphase
 * pulse shaper -> carrier (+ optional guard tone). Drives the full ITU
 * startup training sequence, then streams data from the get_bit callback.
 * Algorithms reimplemented from the V.22bis Rec, with spandsp 0.0.6 as a
 * structural reference; parameters match spandsp so the two interoperate. */

#define V22_LOW_FREQ        1200.0   /* calling modem transmits here   */
#define V22_HIGH_FREQ       2400.0   /* answering modem transmits here */
#define V22_GUARD_550_FREQ   550.0
#define V22_GUARD_1800_FREQ 1800.0

/* Symbols per ms at 600 baud. */
#define MS_TO_SYMBOLS(t)   (((t) * V22_BAUD) / 1000)

/* Differential quadrant code: dibit -> quadrant phase change.
 * 00->+90, 01->0, 10->+180, 11->+270. The table is its own inverse. */
static const int phase_steps[4] = {1, 0, 2, 3};

/* 16-QAM constellation, I/Q in {±1,±3}, grouped by quadrant. Index =
 * (quadrant<<2)|inner2. Matches spandsp byte-for-byte. */
const v22_cf v22_constellation[16] =
{
    { 1.0f,  1.0f}, { 3.0f,  1.0f}, { 1.0f,  3.0f}, { 3.0f,  3.0f},   /* Q0 */
    {-1.0f,  1.0f}, {-1.0f,  3.0f}, {-3.0f,  1.0f}, {-3.0f,  3.0f},   /* Q1 */
    {-1.0f, -1.0f}, {-3.0f, -1.0f}, {-1.0f, -3.0f}, {-3.0f, -3.0f},   /* Q2 */
    { 1.0f, -1.0f}, { 1.0f, -3.0f}, { 3.0f, -1.0f}, { 3.0f, -3.0f}    /* Q3 */
};

/* Scrambler shift taps for an ITU polynomial 1 + x^-a + x^-b:
 * out = bit ^ (reg >> (a-1)) ^ (reg >> (b-1)). */
#define V22_CALLING_TAP1   13   /* 1 + x^-14 + x^-17 */
#define V22_CALLING_TAP2   16
#define V22_ANSWERING_TAP1 17   /* 1 + x^-18 + x^-23 */
#define V22_ANSWERING_TAP2 22

/* ── DDS ──────────────────────────────────────────────────────────────── */

static v22_cf dds_complex(uint32_t *phase, int32_t rate)
{
    double a = (double)(*phase) * (2.0 * M_PI / 4294967296.0);
    *phase += (uint32_t)rate;
    v22_cf z = { (float)cos(a), (float)sin(a) };
    return z;
}

static float dds_real(uint32_t *phase, int32_t rate, float level)
{
    double a = (double)(*phase) * (2.0 * M_PI / 4294967296.0);
    *phase += (uint32_t)rate;
    return level * (float)cos(a);
}

/* ── Scrambler ────────────────────────────────────────────────────────── */

static int scramble(v22bis_t *s, int bit)
{
    if (s->tx.scrambler_ones >= 64) {     /* break a long run of ones */
        bit ^= 1;
        s->tx.scrambler_ones = 0;
    }
    int out = (bit ^ (int)(s->tx.scramble_reg >> s->tx.tap1)
                   ^ (int)(s->tx.scramble_reg >> s->tx.tap2)) & 1;
    s->tx.scramble_reg = (s->tx.scramble_reg << 1) | (uint32_t)out;
    s->tx.scrambler_ones = out ? s->tx.scrambler_ones + 1 : 0;
    return out;
}

/* Pull and scramble one source bit (idle source returns ones). */
static int get_scrambled_bit(v22bis_t *s)
{
    int bit = s->get_bit ? s->get_bit(s->get_user) : 1;
    return scramble(s, bit & 1);
}

/* ── Symbol generation ────────────────────────────────────────────────── */

/* One training symbol; advances the TX training state machine. */
static v22_cf training_symbol(v22bis_t *s)
{
    v22_cf zero = {0.0f, 0.0f};
    int bits;

    switch (s->tx.training) {
    case V22_TX_STAGE_INITIAL_SILENCE:
        /* The answering side sends a brief silence, then unscrambled ones;
         * the calling side stays silent until its receiver drives it on. */
        s->tx.constellation_state = 0;
        if (!s->calling_party && ++s->tx.training_count >= MS_TO_SYMBOLS(75)) {
            s->tx.training_count = 0;
            s->tx.training = V22_TX_STAGE_U11;
        }
        return zero;

    case V22_TX_STAGE_U11:
        /* Continuous unscrambled ones = 270° steps (a pure rotating point). */
        s->tx.constellation_state = (s->tx.constellation_state + phase_steps[3]) & 3;
        return v22_constellation[(s->tx.constellation_state << 2) | 0x01];

    case V22_TX_STAGE_U0011:
        /* S1 segment: unscrambled alternating 00/11 dibits for 100 ms, then
         * scrambled ones. Requests/accepts 2400 bps. */
        s->tx.constellation_state =
            (s->tx.constellation_state + phase_steps[3 * (s->tx.training_count & 1)]) & 3;
        if (++s->tx.training_count >= MS_TO_SYMBOLS(100)) {
            if (s->calling_party) {
                s->tx.training_count = 0;
                s->tx.training = V22_TX_STAGE_S11;
            } else {
                s->tx.training_count = MS_TO_SYMBOLS(756 - (600 - 100));
                s->tx.training = V22_TX_STAGE_TIMED_S11;
            }
        }
        return v22_constellation[(s->tx.constellation_state << 2) | 0x01];

    case V22_TX_STAGE_TIMED_S11:
        /* A timed run of scrambled ones at 1200 bps, then 2400 training or
         * (if we stayed at 1200) data. Falls through to S11 to emit a symbol. */
        if (++s->tx.training_count >= MS_TO_SYMBOLS(756)) {
            s->tx.training_count = 0;
            s->tx.training = (s->negotiated_bit_rate == 2400)
                           ? V22_TX_STAGE_S1111 : V22_TX_STAGE_NORMAL;
        }
        /* fall through */
    case V22_TX_STAGE_S11:
        /* Scrambled ones at 1200 bps (the equalizer-training segment). */
        bits = (scramble(s, 1) << 1) | scramble(s, 1);
        s->tx.constellation_state = (s->tx.constellation_state + phase_steps[bits]) & 3;
        return v22_constellation[(s->tx.constellation_state << 2) | 0x01];

    case V22_TX_STAGE_S1111:
        /* Scrambled ones at 2400 bps: a 200 ms burst, then data. */
        bits = (scramble(s, 1) << 1) | scramble(s, 1);
        s->tx.constellation_state = (s->tx.constellation_state + phase_steps[bits]) & 3;
        bits = (scramble(s, 1) << 1) | scramble(s, 1);
        if (++s->tx.training_count >= MS_TO_SYMBOLS(200)) {
            s->tx.training_count = 0;
            s->tx.training = V22_TX_STAGE_NORMAL;
        }
        return v22_constellation[(s->tx.constellation_state << 2) | bits];

    case V22_TX_STAGE_NORMAL:
    case V22_TX_STAGE_PARKED:
    default:
        return zero;
    }
}

/* One data symbol from the scrambled bit stream. */
static v22_cf data_symbol(v22bis_t *s)
{
    int bits = (get_scrambled_bit(s) << 1) | get_scrambled_bit(s);
    s->tx.constellation_state = (s->tx.constellation_state + phase_steps[bits]) & 3;
    int inner;
    if (s->negotiated_bit_rate == 1200)
        inner = 0x01;                       /* QPSK: one point per quadrant */
    else
        inner = (get_scrambled_bit(s) << 1) | get_scrambled_bit(s);
    int idx = (s->tx.constellation_state << 2) | inner;
    if (s->tx.sym_report) s->tx.sym_report(s->tx.sym_user, idx);
    return v22_constellation[idx];
}

static v22_cf get_symbol(v22bis_t *s)
{
    return s->tx.training ? training_symbol(s) : data_symbol(s);
}

/* ── Init ─────────────────────────────────────────────────────────────── */

v22bis_t *v22bis_init(v22bis_t *s, int max_bit_rate, int calling_party,
                      int guard, int spandsp_compat,
                      v22_get_bit_t get_bit, void *get_user,
                      v22_put_bit_t put_bit, void *put_user,
                      int amplitude)
{
    if (!s) return NULL;
    if (max_bit_rate != 1200 && max_bit_rate != 2400) return NULL;
    if (guard < V22_GUARD_NONE || guard > V22_GUARD_1800) return NULL;

    memset(s, 0, sizeof(*s));
    s->max_bit_rate     = max_bit_rate;
    s->negotiated_bit_rate = 0;
    s->calling_party    = calling_party ? 1 : 0;
    s->guard            = s->calling_party ? V22_GUARD_NONE : guard;
    s->spandsp_compat   = spandsp_compat ? 1 : 0;
    s->amplitude        = (float)amplitude;
    s->get_bit = get_bit;  s->get_user = get_user;
    s->put_bit = put_bit;  s->put_user = put_user;

    /* Scrambler taps. A modem scrambles its TX with its own-direction
     * polynomial and descrambles the far end's with the far end's. compat
     * collapses both to the calling polynomial (matches spandsp). */
    if (s->spandsp_compat) {
        s->tx.tap1 = s->rx.tap1 = V22_CALLING_TAP1;
        s->tx.tap2 = s->rx.tap2 = V22_CALLING_TAP2;
    } else if (s->calling_party) {
        s->tx.tap1 = V22_CALLING_TAP1;   s->tx.tap2 = V22_CALLING_TAP2;
        s->rx.tap1 = V22_ANSWERING_TAP1; s->rx.tap2 = V22_ANSWERING_TAP2;
    } else {
        s->tx.tap1 = V22_ANSWERING_TAP1; s->tx.tap2 = V22_ANSWERING_TAP2;
        s->rx.tap1 = V22_CALLING_TAP1;   s->rx.tap2 = V22_CALLING_TAP2;
    }

    double tx_freq = s->calling_party ? V22_LOW_FREQ  : V22_HIGH_FREQ;
    double rx_freq = s->calling_party ? V22_HIGH_FREQ : V22_LOW_FREQ;
    s->tx.carrier_phase_rate = V22_DDS_RATE(tx_freq);
    s->rx.carrier_phase_rate = V22_DDS_RATE(rx_freq);

    if (s->guard == V22_GUARD_550)
        s->tx.guard_phase_rate = V22_DDS_RATE(V22_GUARD_550_FREQ);
    else if (s->guard == V22_GUARD_1800)
        s->tx.guard_phase_rate = V22_DDS_RATE(V22_GUARD_1800_FREQ);
    s->tx.guard_level = (s->guard == V22_GUARD_NONE) ? 0.0f : s->amplitude * 0.1f;

    /* Output scale: keep the worst-case shaped+modulated sample within int16. */
    s->tx.gain = s->amplitude / V22_TX_SHAPER_PEAK;

    s->tx.training       = V22_TX_STAGE_INITIAL_SILENCE;
    s->tx.training_count = 0;
    s->tx.baud_phase     = 0;

    v22bis_rx_restart(s);
    return s;
}

/* ── Block generation ─────────────────────────────────────────────────── */

int v22bis_tx(v22bis_t *s, int16_t amp[], int len)
{
    for (int n = 0; n < len; n++) {
        /* Symbol clock: 600 baud at 8 kHz via a /40, step-3 polyphase index. */
        if ((s->tx.baud_phase += 3) >= V22_TX_SHAPER_PHASES) {
            s->tx.baud_phase -= V22_TX_SHAPER_PHASES;
            v22_cf sym = get_symbol(s);
            s->tx.rrc[s->tx.rrc_step] = sym;
            s->tx.rrc[s->tx.rrc_step + V22_TX_FILTER_STEPS] = sym;
            if (++s->tx.rrc_step >= V22_TX_FILTER_STEPS) s->tx.rrc_step = 0;
        }

        /* Baseband RRC pulse shaping (polyphase). */
        const float *h = v22_tx_shaper[V22_TX_SHAPER_PHASES - 1 - s->tx.baud_phase];
        v22_cf x = {0.0f, 0.0f};
        for (int i = 0; i < V22_TX_FILTER_STEPS; i++) {
            x.re += h[i] * s->tx.rrc[i + s->tx.rrc_step].re;
            x.im += h[i] * s->tx.rrc[i + s->tx.rrc_step].im;
        }

        /* Modulate onto the carrier, add the guard tone (answering only). */
        v22_cf c = dds_complex(&s->tx.carrier_phase, s->tx.carrier_phase_rate);
        float famp = (x.re * c.re - x.im * c.im) * s->tx.gain;
        if (s->tx.guard_phase_rate &&
            (s->tx.rrc[s->tx.rrc_step].re != 0.0f || s->tx.rrc[s->tx.rrc_step].im != 0.0f))
            famp += dds_real(&s->tx.guard_phase, s->tx.guard_phase_rate, s->tx.guard_level);

        if (famp >  32767.0f) famp =  32767.0f;
        if (famp < -32768.0f) famp = -32768.0f;
        amp[n] = (int16_t)lrintf(famp);
    }
    return len;
}

int v22bis_current_bit_rate(const v22bis_t *s) { return s->negotiated_bit_rate; }
