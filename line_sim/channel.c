#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "channel.h"
#include "../sip_interface/alaw.h"      /* the repo's G.711 A-law (PCMA) path */
#include <math.h>
#include <string.h>
#include <stdio.h>

#define INTERNAL_FS 48000.0
#define NL_REF_AMP  0.30                /* THD specified at this tone level   */
#define PS_LEAK     1e-3                /* signal-power estimator leak (8 kHz) */

/* ── FIFO ───────────────────────────────────────────────────────────────── */

void fifo_init(fifo_t *f) { f->head = f->tail = f->count = 0; }

void fifo_push(fifo_t *f, int16_t v) {
    if (f->count >= FIFO_CAP) {          /* overflow guard: drop oldest */
        f->head = (f->head + 1) % FIFO_CAP;
        f->count--;
    }
    f->buf[f->tail] = v;
    f->tail = (f->tail + 1) % FIFO_CAP;
    f->count++;
}

int fifo_pop(fifo_t *f, int16_t *out, int n) {
    int got = 0;
    while (got < n && f->count > 0) {
        out[got++] = f->buf[f->head];
        f->head = (f->head + 1) % FIFO_CAP;
        f->count--;
    }
    return got;
}

/* ── PRNG helpers ──────────────────────────────────────────────────────── */

static double urand(uint32_t *s) {
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return (double)(x >> 8) * (1.0 / 16777216.0);   /* [0,1) */
}

/* Unit-variance Gaussian via the sum-of-12-uniforms CLT approximation. */
static double grand(uint32_t *s) {
    double a = 0.0;
    for (int i = 0; i < 12; i++) a += urand(s);
    return a - 6.0;
}

static int16_t clip16(double v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return (int16_t)lrint(v);
}

/* ── Non-linear distortion calibration ─────────────────────────────────── */

static double bin_power(const double *x, int n, int bin) {
    double re = 0.0, im = 0.0;
    for (int i = 0; i < n; i++) {
        double a = 2.0 * M_PI * bin * i / n;
        re += x[i] * cos(a);
        im -= x[i] * sin(a);
    }
    return re * re + im * im;
}

/* Signal-to-harmonic-distortion (dB) of y = tanh(g·u)/tanh(g) for a tone of
 * amplitude NL_REF_AMP. tanh is odd, so only odd harmonics appear. */
static double tanh_sdr_db(double g) {
    enum { N = 2400, FB = 50 };
    static double x[N];
    double norm = 1.0 / tanh(g);
    for (int i = 0; i < N; i++) {
        double u = NL_REF_AMP * sin(2.0 * M_PI * FB * i / N);
        x[i] = tanh(g * u) * norm;
    }
    double pf = bin_power(x, N, FB);
    double pd = bin_power(x, N, 3 * FB) + bin_power(x, N, 5 * FB)
              + bin_power(x, N, 7 * FB) + bin_power(x, N, 9 * FB);
    if (pd < 1e-30) pd = 1e-30;
    return 10.0 * log10(pf / pd);
}

/* Find tanh gain g giving the requested signal-to-distortion ratio. SDR is
 * monotonically decreasing in g, so bisect. */
static double calibrate_tanh(double target_db) {
    double lo = 1e-3, hi = 8.0;
    for (int it = 0; it < 60; it++) {
        double g = 0.5 * (lo + hi);
        double s = tanh_sdr_db(g);
        if (s > target_db) lo = g; else hi = g;   /* want lower SDR -> bigger g */
    }
    return 0.5 * (lo + hi);
}

/* ── Frequency-response (tilt) calibration ─────────────────────────────── */

static void build_tilt(chan_t *c, double tilt_db) {
    double g = tilt_db;
    for (int it = 0; it < 4; it++) {
        biquad_high_shelf(&c->fr[0], 1000.0, g, INTERNAL_FS);
        double hi = 20.0 * log10(biquad_mag(&c->fr[0], 3400.0, INTERNAL_FS));
        double lo = 20.0 * log10(biquad_mag(&c->fr[0], 300.0, INTERNAL_FS));
        double d = hi - lo;
        if (fabs(d) < 1e-6) break;
        g *= tilt_db / d;                /* response is ~linear in shelf gain */
    }
    biquad_high_shelf(&c->fr[0], 1000.0, g, INTERNAL_FS);
}

/* ── Group-delay (phase) distortion calibration ────────────────────────── */

static double gd_distortion(const biquad_t *gd, double fs) {
    double e_lo = biquad_chain_group_delay(gd, 2, 350.0, fs);
    double e_hi = biquad_chain_group_delay(gd, 2, 3350.0, fs);
    double mid  = biquad_chain_group_delay(gd, 2, 1800.0, fs);
    double edge = e_lo > e_hi ? e_lo : e_hi;
    return edge - mid;                   /* seconds */
}

static void build_group_delay(chan_t *c, double us) {
    double target = us * 1e-6;
    double lo = 0.05, hi = 0.985;   /* ceiling ~1.5 ms of edge-minus-mid */
    for (int it = 0; it < 50; it++) {
        double r = 0.5 * (lo + hi);
        biquad_allpass2(&c->gd[0], 500.0, r, INTERNAL_FS);
        biquad_allpass2(&c->gd[1], 3000.0, r, INTERNAL_FS);
        double d = gd_distortion(c->gd, INTERNAL_FS);
        if (d < target) lo = r; else hi = r;
    }
    double r = 0.5 * (lo + hi);
    biquad_allpass2(&c->gd[0], 500.0, r, INTERNAL_FS);
    biquad_allpass2(&c->gd[1], 3000.0, r, INTERNAL_FS);
    c->n_gd = 2;
}

/* ── C-message-ish noise weighting (approximate bandpass) ──────────────── */

static void build_cmsg(chan_t *c) {
    /* RBJ constant-0dB bandpass centered ~1700 Hz, broad. */
    double f0 = 1700.0, q = 1.0;
    double w0 = 2.0 * M_PI * f0 / INTERNAL_FS;
    double alpha = sin(w0) / (2.0 * q);
    double a0 = 1.0 + alpha;
    c->cmsg.b0 = alpha / a0;
    c->cmsg.b1 = 0.0;
    c->cmsg.b2 = -alpha / a0;
    c->cmsg.a1 = (-2.0 * cos(w0)) / a0;
    c->cmsg.a2 = (1.0 - alpha) / a0;
    biquad_reset(&c->cmsg);
    /* Measure the filter's white-noise power gain so we can renormalize. */
    uint32_t rng = 0x13572468u;
    double sum = 0.0;
    int n = 60000;
    for (int i = 0; i < n; i++) {
        double y = biquad_run(&c->cmsg, grand(&rng));
        if (i > 1000) sum += y * y;       /* skip startup transient */
    }
    double var = sum / (n - 1000);
    c->cmsg_scale = var > 0 ? 1.0 / sqrt(var) : 1.0;
    biquad_reset(&c->cmsg);
}

/* ── FIR mask loading ──────────────────────────────────────────────────── */

static int load_fir(chan_t *c, const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int n = 0;
    double v;
    while (n < LS_MAX_FIR && fscanf(f, "%lf", &v) == 1) c->fir[n++] = v;
    fclose(f);
    if (n == 0) return -1;
    c->fir_len = n;
    return 0;
}

/* ── Init / reset ──────────────────────────────────────────────────────── */

int chan_init(chan_t *c, const chan_cfg_t *cfg, int ext_rate, uint32_t seed) {
    memset(c, 0, sizeof(*c));
    c->ext_rate = ext_rate;
    if (resamp_init(&c->rs, ext_rate) != 0) return -1;

    c->gain_lin = pow(10.0, cfg->gain_db / 20.0);

    /* Frequency response: high-shelf tilt (fr[0]) then peaking bumps. */
    c->n_fr = 0;
    if (fabs(cfg->freq_tilt_db) > 1e-9) {
        build_tilt(c, cfg->freq_tilt_db);
        c->n_fr = 1;
    }
    for (int i = 0; i < cfg->nbumps && i < LS_MAX_BUMPS; i++) {
        biquad_peaking(&c->fr[c->n_fr], cfg->bump[i].f0, cfg->bump[i].q,
                       cfg->bump[i].gain_db, INTERNAL_FS);
        c->n_fr++;
    }
    if (cfg->fir_path) {
        if (load_fir(c, cfg->fir_path) != 0) return -1;
    }

    if (cfg->gd_us > 1e-9) build_group_delay(c, cfg->gd_us);

    if (cfg->has_thd) {
        c->nl_on = 1;
        c->nl_g = calibrate_tanh(cfg->thd_db);
        c->nl_norm = 1.0 / tanh(c->nl_g);
    }

    if (cfg->offset_hz != 0.0 || cfg->jitter_deg != 0.0) {
        c->oj_on = 1;
        hilbert_init(&c->hb);
        c->off_step = 2.0 * M_PI * cfg->offset_hz / INTERNAL_FS;
        c->jit_peak = cfg->jitter_deg * M_PI / 180.0;
        c->jit_step = 2.0 * M_PI * cfg->jitter_rate_hz / INTERNAL_FS;
    }

    if (cfg->has_snr) {
        c->noise_on = 1;
        c->snr_lin = pow(10.0, cfg->snr_db / 10.0);
        c->noise_cmsg = cfg->noise_cmsg;
        if (c->noise_cmsg) build_cmsg(c);
        c->nrng = seed ? seed : 0xc0ffeeu;
    }

    c->alaw = cfg->alaw;
    if (c->alaw) alaw_init();

    c->slip_prob = cfg->slip_prob;
    c->srng = (seed ? seed : 0x9e3779b9u) ^ 0x5bd1e995u;

    return 0;
}

void chan_reset(chan_t *c) {
    resamp_reset(&c->rs);
    for (int i = 0; i < c->n_fr; i++) biquad_reset(&c->fr[i]);
    for (int i = 0; i < c->n_gd; i++) biquad_reset(&c->gd[i]);
    if (c->oj_on) hilbert_reset(&c->hb);
    if (c->noise_cmsg) biquad_reset(&c->cmsg);
    memset(c->fir_hist, 0, sizeof(c->fir_hist));
    c->fir_pos = 0;
    c->off_phase = c->jit_phase = 0.0;
    c->ps = 0.0;
    c->slip_accum = 0;
    c->in_count = c->out_count = 0;
    c->slip_min = c->slip_max = 0;
}

/* ── Clock slip ────────────────────────────────────────────────────────── */

static void emit_with_slip(chan_t *c, int16_t v, fifo_t *out) {
    if (c->slip_prob > 0.0 && urand(&c->srng) < c->slip_prob) {
        int want_dup = urand(&c->srng) < 0.5;
        if (want_dup && c->slip_accum < SLIP_MAX) {
            fifo_push(out, v); fifo_push(out, v);
            c->slip_accum++;
        } else if (!want_dup && c->slip_accum > -SLIP_MAX) {
            c->slip_accum--;                       /* drop: emit nothing */
        } else if (want_dup && c->slip_accum > -SLIP_MAX) {
            c->slip_accum--;                       /* bound hit -> do the other */
        } else if (!want_dup && c->slip_accum < SLIP_MAX) {
            fifo_push(out, v); fifo_push(out, v);
            c->slip_accum++;
        } else {
            fifo_push(out, v);                     /* both bounds hit (n/a) */
        }
        if (c->slip_accum < c->slip_min) c->slip_min = c->slip_accum;
        if (c->slip_accum > c->slip_max) c->slip_max = c->slip_accum;
        return;
    }
    fifo_push(out, v);
}

/* ── Per-direction processing ──────────────────────────────────────────── */

void chan_process(chan_t *c, const int16_t *in, int n, fifo_t *out) {
    double up[RS_L_MAX];
    for (int i = 0; i < n; i++) {
        c->in_count++;
        int L = resamp_up(&c->rs, (double)in[i], up);
        for (int j = 0; j < L; j++) {
            double s = up[j];

            s *= c->gain_lin;

            for (int b = 0; b < c->n_fr; b++) s = biquad_run(&c->fr[b], s);

            if (c->fir_len) {
                c->fir_pos = (c->fir_pos + 1) % c->fir_len;
                c->fir_hist[c->fir_pos] = s;
                double acc = 0.0;
                int idx = c->fir_pos;
                for (int k = 0; k < c->fir_len; k++) {
                    acc += c->fir[k] * c->fir_hist[idx];
                    idx = (idx - 1 + c->fir_len) % c->fir_len;
                }
                s = acc;
            }

            for (int b = 0; b < c->n_gd; b++) s = biquad_run(&c->gd[b], s);

            if (c->nl_on) {
                double u = s / 32768.0;
                s = tanh(c->nl_g * u) * c->nl_norm * 32768.0;
            }

            if (c->oj_on) {
                double re, im;
                hilbert_run(&c->hb, s, &re, &im);
                double theta = c->off_phase + c->jit_peak * sin(c->jit_phase);
                s = re * cos(theta) - im * sin(theta);
                c->off_phase += c->off_step;
                if (c->off_phase > M_PI) c->off_phase -= 2.0 * M_PI;
                if (c->off_phase < -M_PI) c->off_phase += 2.0 * M_PI;
                c->jit_phase += c->jit_step;
                if (c->jit_phase > 2.0 * M_PI) c->jit_phase -= 2.0 * M_PI;
            }

            double ds;
            if (resamp_down(&c->rs, s, &ds)) {
                if (c->alaw) ds = (double)alaw_decode(alaw_encode(clip16(ds)));

                if (c->noise_on) {
                    c->ps += PS_LEAK * (ds * ds - c->ps);
                    double pn = c->ps / c->snr_lin;
                    double nz = grand(&c->nrng);
                    if (c->noise_cmsg) nz = biquad_run(&c->cmsg, nz) * c->cmsg_scale;
                    ds += sqrt(pn) * nz;
                }

                c->out_count++;
                emit_with_slip(c, clip16(ds), out);
            }
        }
    }
}
