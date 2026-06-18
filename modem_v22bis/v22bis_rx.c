#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "v22bis.h"
#include "v22bis_rrc.h"
#include <string.h>
#include <math.h>

/* ── V.22bis receiver ──────────────────────────────────────────────────
 * Front end: NCO carrier demod -> complex RRC matched filter (8 kHz) ->
 * Gardner-driven fractional resampler to T/2 symbols. Back end: T/2
 * fractionally-spaced complex LMS equalizer -> 16-QAM/QPSK slicer ->
 * differential decode -> descrambler, with a decision-directed carrier PI
 * loop. Per-baud processing and loop structure mirror spandsp 0.0.6. */

_Static_assert(sizeof(v22_rx_mf) / sizeof(v22_rx_mf[0]) == V22_RX_MF_TAPS,
               "v22bis_rrc.h matched filter length must match V22_RX_MF_TAPS");

#define EQ_TAPS        (2 * V22_EQ_LEN + 1)
#define NOMINAL_HALF   ((double)V22_SAMPLE_RATE / V22_BAUD / 2.0)  /* 6.667 */
#define WARMUP_SAMPLES  150     /* AGC settle after carrier-up (fast) */
#define ACQUIRE_SYMBOLS 40      /* symbols of timing pull-in (spec-paced) */
#define TARGET_RMS      3.162f  /* |symbol| RMS of the 16-QAM constellation   */

/* Loop gains (tunable). */
#define MF_POWER_LEAK   0.05f
#define MF_POWER_LEAK_SLOW 0.0008f
#define EQ_DELTA        0.5f
#define CARRIER_KP      0.004
#define CARRIER_KI      0.00003
#define GARDNER_KP      0.02
#define GARDNER_KI      0.002
#define POWER_LEAK      0.01f
#define CARRIER_RATE_TOL 0.02   /* fractional clamp around the nominal rate    */

static const int phase_steps[4] = {1, 0, 2, 3};

/* ── Complex helpers ──────────────────────────────────────────────────── */
static inline v22_cf cf_mul(v22_cf a, v22_cf b) {
    v22_cf r = { a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re };
    return r;
}

/* ── Descrambler (inverse of the TX scrambler; see v22bis_tx.c) ─────────── */
static int descramble(v22bis_t *s, int bit)
{
    bit &= 1;
    int out = (bit ^ (int)(s->rx.scramble_reg >> s->rx.tap1)
                   ^ (int)(s->rx.scramble_reg >> s->rx.tap2)) & 1;
    s->rx.scramble_reg = (s->rx.scramble_reg << 1) | (uint32_t)bit;
    if (s->rx.scrambler_ones >= 64) { out ^= 1; s->rx.scrambler_ones = 0; }
    s->rx.scrambler_ones = bit ? s->rx.scrambler_ones + 1 : 0;
    return out;
}

/* ── Slicer ───────────────────────────────────────────────────────────── */

/* Nearest constellation index to z. In QPSK mode only the four inner=01
 * points (one per quadrant) are valid. */
static int slice(const v22bis_t *s, v22_cf z)
{
    int best = 0;
    float bestd = 1e30f;
    if (s->rx.sixteen_way) {
        for (int i = 0; i < 16; i++) {
            float dr = z.re - v22_constellation[i].re;
            float di = z.im - v22_constellation[i].im;
            float d = dr * dr + di * di;
            if (d < bestd) { bestd = d; best = i; }
        }
    } else {
        for (int q = 0; q < 4; q++) {
            int i = (q << 2) | 0x01;
            float dr = z.re - v22_constellation[i].re;
            float di = z.im - v22_constellation[i].im;
            float d = dr * dr + di * di;
            if (d < bestd) { bestd = d; best = i; }
        }
    }
    return best;
}

/* ── Equalizer ────────────────────────────────────────────────────────── */

static v22_cf equalizer_get(v22bis_t *s)
{
    v22_cf z = {0.0f, 0.0f};
    int p = s->rx.eq_step - 1;
    for (int i = 0; i < EQ_TAPS; i++) {
        p = (p - 1) & V22_EQ_MASK;
        v22_cf t = cf_mul(s->rx.eq_coeff[i], s->rx.eq_buf[p]);
        z.re += t.re; z.im += t.im;
    }
    return z;
}

static void equalizer_tune(v22bis_t *s, v22_cf z, v22_cf target)
{
    /* Normalised LMS: divide the step by the tap-line input energy so the
     * update is scale-invariant. This keeps the equalizer stable through the
     * AGC re-levelling transient when training ones give way to wideband data
     * (a plain LMS diverges there if the input scale is briefly off). */
    float energy = 1e-3f;
    int p = s->rx.eq_step - 1;
    for (int i = 0; i < EQ_TAPS; i++) {
        p = (p - 1) & V22_EQ_MASK;
        energy += s->rx.eq_buf[p].re * s->rx.eq_buf[p].re
                + s->rx.eq_buf[p].im * s->rx.eq_buf[p].im;
    }
    float mu = s->rx.eq_delta / energy;
    v22_cf ez = { (target.re - z.re) * mu, (target.im - z.im) * mu };
    p = s->rx.eq_step - 1;
    for (int i = 0; i < EQ_TAPS; i++) {
        p = (p - 1) & V22_EQ_MASK;
        v22_cf cj = { s->rx.eq_buf[p].re, -s->rx.eq_buf[p].im };  /* conj */
        v22_cf d = cf_mul(ez, cj);
        s->rx.eq_coeff[i].re = (s->rx.eq_coeff[i].re + d.re) * 0.9999f;
        s->rx.eq_coeff[i].im = (s->rx.eq_coeff[i].im + d.im) * 0.9999f;
    }
}

/* ── Carrier tracking (decision-directed PI loop) ─────────────────────── */
static void track_carrier(v22bis_t *s, v22_cf z, v22_cf target)
{
    /* Imag part of z * conj(target) ~ phase error, scaled by |target|^2. */
    double error = (double)z.im * target.re - (double)z.re * target.im;
    double n = target.re * target.re + target.im * target.im;
    if (n > 0.0) error /= n;
    /* Our NCO derotates with e^{+j.phase} and a negative rate, the opposite
     * sign convention to spandsp, so corrections subtract. */
    s->rx.carrier_phase_rate -= s->rx.carrier_ki * error;
    s->rx.carrier_phase      -= s->rx.carrier_kp * error;
    /* Keep the recovered rate within a small band of the nominal carrier. */
    double nom = -2.0 * M_PI * (s->calling_party ? 2400.0 : 1200.0) / V22_SAMPLE_RATE;
    double lo = nom * (1.0 + CARRIER_RATE_TOL), hi = nom * (1.0 - CARRIER_RATE_TOL);
    if (s->rx.carrier_phase_rate < lo) s->rx.carrier_phase_rate = lo;
    if (s->rx.carrier_phase_rate > hi) s->rx.carrier_phase_rate = hi;
}

/* ── Gardner timing-error detector (operates on T/2 eq_buf samples) ─────── */
static void symbol_timing(v22bis_t *s)
{
    int m = V22_EQ_MASK;
    v22_cf a = s->rx.eq_buf[(s->rx.eq_step - 3) & m];   /* early */
    v22_cf b = s->rx.eq_buf[(s->rx.eq_step - 2) & m];   /* mid   */
    v22_cf c = s->rx.eq_buf[(s->rx.eq_step - 1) & m];   /* late  */
    double e = (double)(c.re - a.re) * b.re + (double)(c.im - a.im) * b.im;
    /* Normalise by the mid-sample energy so the error is scale-invariant, and
     * clamp it so a transient can't kick the timing instant out of range. */
    double nrm = (double)b.re * b.re + (double)b.im * b.im + 1.0;
    e /= nrm;
    if (e >  1.0) e =  1.0;
    if (e < -1.0) e = -1.0;
    /* Second-order timing loop: nudge the resample instant and the period. */
    s->rx.next_t      += s->rx.gardner_kp * e;
    s->rx.half_period += s->rx.gardner_ki * e;
    if (s->rx.half_period < NOMINAL_HALF - 0.3) s->rx.half_period = NOMINAL_HALF - 0.3;
    if (s->rx.half_period > NOMINAL_HALF + 0.3) s->rx.half_period = NOMINAL_HALF + 0.3;
}

/* ── Per-baud processing ──────────────────────────────────────────────── */

#define MS_SYM(t)  (((t) * V22_BAUD) / 1000)

/* Differential-decode + descramble one symbol, updating constellation_state.
 * Returns the recovered bitstream (2 bits QPSK, 4 bits 16-QAM). Emits via
 * put_bit only when emit != 0 (training segments descramble but don't emit, to
 * keep the descrambler synced for the data that follows). */
static int decode_baud(v22bis_t *s, int nearest, int emit)
{
    int raw = phase_steps[((nearest >> 2) - (s->rx.constellation_state >> 2)) & 3];
    s->rx.constellation_state = nearest;
    int b1 = descramble(s, raw >> 1);
    int b0 = descramble(s, raw & 1);
    int out = (b1 << 1) | b0;
    if (emit && s->put_bit) { s->put_bit(s->put_user, b1); s->put_bit(s->put_user, b0); }
    if (s->rx.sixteen_way) {
        int i1 = descramble(s, (nearest >> 1) & 1);
        int i0 = descramble(s, nearest & 1);
        out = (out << 2) | (i1 << 1) | i0;
        if (emit && s->put_bit) { s->put_bit(s->put_user, i1); s->put_bit(s->put_user, i0); }
    }
    return out;
}

/* Process one whole baud, running the coupled V.22bis startup state machine
 * (mirrors spandsp): the receiver's progress drives the local transmitter's
 * training stage and the negotiated rate, so two modems hand-shake to NORMAL.
 * In SYMBOL_ACQUISITION only timing/AGC run; carrier and equalizer adapt once
 * a constellation reference exists. */
static void process_baud(v22bis_t *s)
{
    symbol_timing(s);
    v22_cf z = equalizer_get(s);
    int nearest = slice(s, z);
    v22_cf target = v22_constellation[nearest];
    int raw_bits = phase_steps[((nearest >> 2) - (s->rx.constellation_state >> 2)) & 3];
    int twobit = s->max_bit_rate == 2400;
    int bitstream;

    switch (s->rx.training) {
    case V22_RX_STAGE_SYMBOL_ACQUISITION:
        /* Pull in timing (and AGC, in the sample loop); no carrier/EQ yet. */
        if (++s->rx.training_count >= ACQUIRE_SYMBOLS) {
            s->negotiated_bit_rate = 1200;
            s->rx.pattern_repeats = 0;
            s->rx.constellation_state = nearest;
            s->rx.training_count = 0;
            s->rx.training = s->calling_party
                ? V22_RX_STAGE_UNSCRAMBLED_ONES
                : V22_RX_STAGE_SCRAMBLED_ONES_AT_1200;
        }
        break;

    case V22_RX_STAGE_UNSCRAMBLED_ONES:          /* calling only */
        track_carrier(s, z, target);
        equalizer_tune(s, z, target);
        s->rx.constellation_state = nearest;
        s->rx.pattern_repeats = (raw_bits == s->rx.last_raw_bits)
                              ? s->rx.pattern_repeats + 1 : 0;
        if (++s->rx.training_count == MS_SYM(155 + 456)) {
            if (raw_bits == s->rx.last_raw_bits &&
                (raw_bits == 0x3 || raw_bits == 0x0) &&
                s->rx.pattern_repeats >= MS_SYM(456)) {
                /* Steady unscrambled ones from the answerer: respond. */
                s->tx.training_count = 0;
                s->tx.training = twobit ? V22_TX_STAGE_U0011 : V22_TX_STAGE_S11;
            }
            s->rx.pattern_repeats = 0;
            s->rx.training_count = 0;
            s->rx.training = V22_RX_STAGE_UNSCRAMBLED_ONES_SUSTAINING;
        }
        break;

    case V22_RX_STAGE_UNSCRAMBLED_ONES_SUSTAINING:   /* calling only */
        track_carrier(s, z, target);
        equalizer_tune(s, z, target);
        s->rx.constellation_state = nearest;
        if (raw_bits != s->rx.last_raw_bits) {       /* end of unscrambled ones */
            s->tx.training_count = 0; s->tx.training = V22_TX_STAGE_TIMED_S11;
            s->rx.training_count = 0; s->rx.pattern_repeats = 0;
            s->rx.training = V22_RX_STAGE_SCRAMBLED_ONES_AT_1200;
            /* Re-level the AGC on the wideband scrambled ones (set on the U11
             * tone); the normalised LMS keeps the equalizer stable. */
            s->rx.agc_warm = WARMUP_SAMPLES; s->rx.mf_power = 0.0f;
        }
        break;

    case V22_RX_STAGE_SCRAMBLED_ONES_AT_1200:
        track_carrier(s, z, target);
        equalizer_tune(s, z, target);
        bitstream = decode_baud(s, nearest, 0);
        (void)bitstream;
        s->rx.training_count++;
        if (s->negotiated_bit_rate == 1200) {
            /* Look for the S1 segment (alternating 00/11). */
            if ((s->rx.last_raw_bits ^ raw_bits) == 0x3) {
                s->rx.pattern_repeats++;
            } else {
                if (s->rx.pattern_repeats >= 15 &&
                    (s->rx.last_raw_bits == 0x3 || s->rx.last_raw_bits == 0x0)) {
                    if (twobit) {
                        if (!s->calling_party) {
                            s->tx.training_count = 0;
                            s->tx.training = V22_TX_STAGE_U0011;
                        }
                        s->negotiated_bit_rate = 2400;
                    }
                }
                s->rx.pattern_repeats = 0;
            }
            if (s->rx.training_count >= MS_SYM(270)) {
                s->tx.training_count = 0; s->tx.training = V22_TX_STAGE_TIMED_S11;
                if (s->calling_party) {
                    s->rx.training = V22_RX_STAGE_NORMAL;
                } else {
                    s->rx.training = V22_RX_STAGE_SCRAMBLED_ONES_AT_1200_SUSTAINING;
                }
            }
        } else {  /* heading for 2400 */
            int due = s->calling_party ? MS_SYM(100 + 450) : MS_SYM(450);
            if (s->rx.training_count >= due) {
                s->rx.sixteen_way = 1;
                s->rx.pattern_repeats = 0;
                s->rx.training = V22_RX_STAGE_WAIT_FOR_SCRAMBLED_ONES_AT_2400;
                /* Re-calibrate the AGC on the wideband signal just as we begin
                 * 16-way decisions, so the 16-QAM constellation is correctly
                 * scaled (the initial calibration may have been on a tone). */
                s->rx.agc_warm = WARMUP_SAMPLES; s->rx.mf_power = 0.0f;
            }
        }
        break;

    case V22_RX_STAGE_SCRAMBLED_ONES_AT_1200_SUSTAINING:   /* answerer 1200 */
        track_carrier(s, z, target);
        equalizer_tune(s, z, target);
        decode_baud(s, nearest, 0);
        if (++s->rx.training_count > MS_SYM(270 + 765))
            s->rx.training = V22_RX_STAGE_NORMAL;
        break;

    case V22_RX_STAGE_WAIT_FOR_SCRAMBLED_ONES_AT_2400:
        track_carrier(s, z, target);
        equalizer_tune(s, z, target);
        bitstream = decode_baud(s, nearest, 0);
        if (bitstream == 0xF) {     /* 32 consecutive scrambled ones (9 bauds) */
            if (++s->rx.pattern_repeats >= 9)
                s->rx.training = V22_RX_STAGE_NORMAL;
        } else {
            s->rx.pattern_repeats = 0;
        }
        break;

    case V22_RX_STAGE_NORMAL:
        track_carrier(s, z, target);
        equalizer_tune(s, z, target);
        if (s->rx.qam_report) s->rx.qam_report(s->rx.qam_user, z.re, z.im, nearest);
        decode_baud(s, nearest, 1);
        break;

    default:
        break;
    }
    s->rx.last_raw_bits = raw_bits;
}

/* ── Lifecycle ────────────────────────────────────────────────────────── */

void v22bis_rx_restart(v22bis_t *s)
{
    /* Preserve the descrambler taps set up in v22bis_init. */
    int tap1 = s->rx.tap1, tap2 = s->rx.tap2;
    double rx_freq = s->calling_party ? 2400.0 : 1200.0;

    memset(&s->rx, 0, sizeof(s->rx));
    s->rx.tap1 = tap1; s->rx.tap2 = tap2;

    /* NCO runs at -carrier so multiplying brings the signal to baseband. */
    s->rx.carrier_phase_rate = -2.0 * M_PI * rx_freq / V22_SAMPLE_RATE;
    s->rx.agc = 1.0f;
    s->rx.half_period = NOMINAL_HALF;
    s->rx.next_t = NOMINAL_HALF;        /* first T/2 sample after one half     */
    s->rx.eq_coeff[V22_EQ_LEN].re = 1.0f;   /* centre tap unity                 */
    s->rx.eq_delta = EQ_DELTA;
    s->rx.carrier_kp = CARRIER_KP;
    s->rx.carrier_ki = CARRIER_KI;
    s->rx.gardner_kp = GARDNER_KP;
    s->rx.gardner_ki = GARDNER_KI;
    s->rx.training = V22_RX_STAGE_SYMBOL_ACQUISITION;
    s->rx.carrier_present = 0;
}

/* ── Block processing ─────────────────────────────────────────────────── */

/* (Re)start acquisition when a carrier appears, keeping the free-running NCO
 * and resampler clock (y_count) continuous. */
static void acquire_reset(v22bis_t *s)
{
    s->rx.acq = 0;
    s->rx.agc_warm = WARMUP_SAMPLES;
    s->rx.mf_power = 0.0f;
    s->rx.training = V22_RX_STAGE_SYMBOL_ACQUISITION;
    s->rx.training_count = 0;
    s->rx.next_t = (double)s->rx.y_count + NOMINAL_HALF;
    s->rx.half_period = NOMINAL_HALF;
    s->rx.half = 0;
    s->rx.constellation_state = 0;
    s->rx.pattern_repeats = 0;
    s->rx.last_raw_bits = 0;
    s->rx.sixteen_way = 0;
    s->negotiated_bit_rate = 1200;
    s->rx.scramble_reg = 0;
    s->rx.scrambler_ones = 0;
    s->rx.carrier_phase_rate = -2.0 * M_PI * (s->calling_party ? 2400.0 : 1200.0)
                             / V22_SAMPLE_RATE;
    memset(s->rx.eq_coeff, 0, sizeof(s->rx.eq_coeff));
    s->rx.eq_coeff[V22_EQ_LEN].re = 1.0f;
    memset(s->rx.eq_buf, 0, sizeof(s->rx.eq_buf));
    s->rx.eq_step = 0;
}

int v22bis_rx(v22bis_t *s, const int16_t amp[], int len)
{
    for (int n = 0; n < len; n++) {
        float x = (float)amp[n];

        /* Carrier detect from input energy, with hysteresis. The handshake has
         * silent gaps; running the loops on silence would blow up the AGC. */
        s->rx.power += POWER_LEAK * (x * x - s->rx.power);
        if (!s->rx.carrier_present) {
            if (s->rx.power > 1.0e6f) { s->rx.carrier_present = 1; acquire_reset(s); }
            else { continue; }                 /* silence: skip processing */
        } else if (s->rx.power < 2.5e5f) {
            s->rx.carrier_present = 0;
            continue;
        }

        /* NCO demod to baseband. */
        double cph = s->rx.carrier_phase;
        v22_cf bb = { x * (float)cos(cph), x * (float)sin(cph) };
        s->rx.carrier_phase += s->rx.carrier_phase_rate;
        if (s->rx.carrier_phase > 1e7 || s->rx.carrier_phase < -1e7)
            s->rx.carrier_phase = fmod(s->rx.carrier_phase, 2.0 * M_PI);

        /* Complex matched filter (real coeffs). */
        s->rx.mf_buf[s->rx.mf_pos] = bb;
        s->rx.mf_pos = (s->rx.mf_pos + 1) % V22_RX_MF_TAPS;
        v22_cf y = {0.0f, 0.0f};
        int p = s->rx.mf_pos;
        for (int i = 0; i < V22_RX_MF_TAPS; i++) {
            p = (p - 1 + V22_RX_MF_TAPS) % V22_RX_MF_TAPS;
            y.re += v22_rx_mf[i] * s->rx.mf_buf[p].re;
            y.im += v22_rx_mf[i] * s->rx.mf_buf[p].im;
        }

        /* AGC: set the gain from the average matched-filter power during the
         * post-carrier warm-up, then FREEZE it. A gain that keeps tracking
         * would fight the legitimate symbol-to-symbol amplitude swing of
         * 16-QAM and compress the constellation. */
        float mag2 = y.re * y.re + y.im * y.im;
        /* Fast settle right after carrier-up, then a slow track through training
         * (the slow time constant >> a symbol doesn't fight 16-QAM, but re-levels
         * when the tone-like training ones give way to wideband symbols), then
         * FREEZE in normal operation so it can't wobble on random data. */
        if (s->rx.training != V22_RX_STAGE_NORMAL) {
            float leak = (s->rx.agc_warm > 0) ? MF_POWER_LEAK : MF_POWER_LEAK_SLOW;
            if (s->rx.agc_warm > 0) s->rx.agc_warm--;
            s->rx.mf_power += leak * (mag2 - s->rx.mf_power);
            s->rx.agc = TARGET_RMS / sqrtf(s->rx.mf_power + 1e-6f);
        }
        y.re *= s->rx.agc; y.im *= s->rx.agc;

        /* Push into the interpolation history. */
        s->rx.y_hist[s->rx.y_count & 3] = y;
        s->rx.y_count++;

        /* Hold off the loops until the AGC has settled after carrier-up. */
        if (s->rx.acq++ < WARMUP_SAMPLES) {
            s->rx.next_t = (double)s->rx.y_count + NOMINAL_HALF;
            continue;
        }

        /* Emit T/2 samples at the Gardner-tracked instants. */
        while ((double)(s->rx.y_count - 1) >= s->rx.next_t + 1.0) {
            int i0 = (int)floor(s->rx.next_t);
            double frac = s->rx.next_t - i0;
            int back0 = (s->rx.y_count - 1) - i0;
            if (back0 < 1 || back0 > 3) { s->rx.next_t += s->rx.half_period; continue; }
            v22_cf y0 = s->rx.y_hist[i0 & 3];
            v22_cf y1 = s->rx.y_hist[(i0 + 1) & 3];
            v22_cf ts = { (float)(y0.re + frac * (y1.re - y0.re)),
                          (float)(y0.im + frac * (y1.im - y0.im)) };
            s->rx.eq_buf[s->rx.eq_step] = ts;
            s->rx.eq_step = (s->rx.eq_step + 1) & V22_EQ_MASK;
            s->rx.next_t += s->rx.half_period;
            if (s->rx.half ^= 1) {}        /* toggled below */
            if ((s->rx.half) == 0)         /* whole baud ready */
                process_baud(s);
        }
    }
    return len;
}

int v22bis_rx_trained(const v22bis_t *s) { return s->rx.training == V22_RX_STAGE_NORMAL; }
int v22bis_rx_carrier(const v22bis_t *s) { return s->rx.carrier_present; }
