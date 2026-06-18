#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "filters.h"
#include <math.h>

/* ── Biquad ─────────────────────────────────────────────────────────────── */

void biquad_reset(biquad_t *f) { f->z1 = f->z2 = 0.0; }

double biquad_run(biquad_t *f, double x) {
    double y = f->b0 * x + f->z1;
    f->z1 = f->b1 * x - f->a1 * y + f->z2;
    f->z2 = f->b2 * x - f->a2 * y;
    return y;
}

double biquad_mag(const biquad_t *f, double hz, double fs) {
    double w = 2.0 * M_PI * hz / fs;
    double cw = cos(w), sw = sin(w), c2 = cos(2 * w), s2 = sin(2 * w);
    /* H = (b0 + b1 e^-jw + b2 e^-2jw) / (1 + a1 e^-jw + a2 e^-2jw) */
    double nr = f->b0 + f->b1 * cw + f->b2 * c2;
    double ni = -(f->b1 * sw + f->b2 * s2);
    double dr = 1.0 + f->a1 * cw + f->a2 * c2;
    double di = -(f->a1 * sw + f->a2 * s2);
    double num = sqrt(nr * nr + ni * ni);
    double den = sqrt(dr * dr + di * di);
    return den > 0 ? num / den : 0.0;
}

/* Phase response (radians) of a biquad at hz. */
static double biquad_phase(const biquad_t *f, double hz, double fs) {
    double w = 2.0 * M_PI * hz / fs;
    double cw = cos(w), sw = sin(w), c2 = cos(2 * w), s2 = sin(2 * w);
    double nr = f->b0 + f->b1 * cw + f->b2 * c2;
    double ni = -(f->b1 * sw + f->b2 * s2);
    double dr = 1.0 + f->a1 * cw + f->a2 * c2;
    double di = -(f->a1 * sw + f->a2 * s2);
    return atan2(ni, nr) - atan2(di, dr);
}

void biquad_peaking(biquad_t *f, double hz, double q, double gain_db, double fs) {
    double A = pow(10.0, gain_db / 40.0);
    double w0 = 2.0 * M_PI * hz / fs;
    double cw = cos(w0), sw = sin(w0);
    double alpha = sw / (2.0 * q);
    double a0 = 1.0 + alpha / A;
    f->b0 = (1.0 + alpha * A) / a0;
    f->b1 = (-2.0 * cw) / a0;
    f->b2 = (1.0 - alpha * A) / a0;
    f->a1 = (-2.0 * cw) / a0;
    f->a2 = (1.0 - alpha / A) / a0;
    biquad_reset(f);
}

void biquad_high_shelf(biquad_t *f, double hz, double gain_db, double fs) {
    double A = pow(10.0, gain_db / 40.0);
    double w0 = 2.0 * M_PI * hz / fs;
    double cw = cos(w0), sw = sin(w0);
    double alpha = sw / 2.0 * sqrt(2.0);        /* shelf slope S = 1 */
    double tsa = 2.0 * sqrt(A) * alpha;
    double a0 =        (A + 1.0) - (A - 1.0) * cw + tsa;
    f->b0 =      A * ( (A + 1.0) + (A - 1.0) * cw + tsa) / a0;
    f->b1 = -2.0 * A * ( (A - 1.0) + (A + 1.0) * cw)      / a0;
    f->b2 =      A * ( (A + 1.0) + (A - 1.0) * cw - tsa) / a0;
    f->a1 =  2.0 * ( (A - 1.0) - (A + 1.0) * cw)         / a0;
    f->a2 =      ( (A + 1.0) - (A - 1.0) * cw - tsa)     / a0;
    biquad_reset(f);
}

void biquad_allpass2(biquad_t *f, double hz, double r, double fs) {
    double w0 = 2.0 * M_PI * hz / fs;
    double a1 = -2.0 * r * cos(w0);
    double a2 = r * r;
    /* All-pass: numerator is the denominator reversed -> unity magnitude. */
    f->b0 = a2;
    f->b1 = a1;
    f->b2 = 1.0;
    f->a1 = a1;
    f->a2 = a2;
    biquad_reset(f);
}

double biquad_chain_group_delay(const biquad_t *secs, int n, double hz, double fs) {
    /* tau = -dphase/dw, via central difference in frequency. */
    double dh = 1.0;                            /* 1 Hz step */
    double p1 = 0.0, p2 = 0.0;
    for (int i = 0; i < n; i++) {
        p1 += biquad_phase(&secs[i], hz - dh, fs);
        p2 += biquad_phase(&secs[i], hz + dh, fs);
    }
    double dphase = p2 - p1;
    /* Unwrap the small step. */
    while (dphase >  M_PI) dphase -= 2.0 * M_PI;
    while (dphase < -M_PI) dphase += 2.0 * M_PI;
    /* Group delay in seconds = -dphase / d(omega_physical), with
     * omega_physical = 2*pi*f (rad/s); d(omega) = 2*pi*(2*dh). No fs factor. */
    (void)fs;
    double dw = 2.0 * M_PI * (2.0 * dh);
    return -dphase / dw;
}

/* ── Hilbert transformer ───────────────────────────────────────────────── */

void hilbert_init(hilbert_t *hb) {
    int M = (HILBERT_TAPS - 1) / 2;
    for (int i = 0; i < HILBERT_TAPS; i++) {
        int k = i - M;
        double v;
        if (k % 2 == 0) {
            v = 0.0;                            /* even offsets are zero */
        } else {
            v = 2.0 / (M_PI * k);               /* ideal Hilbert kernel */
        }
        /* Hamming window for a clean ~300-3400 Hz quadrature response. */
        double win = 0.54 - 0.46 * cos(2.0 * M_PI * i / (HILBERT_TAPS - 1));
        hb->h[i] = v * win;
    }
    hilbert_reset(hb);
}

void hilbert_reset(hilbert_t *hb) {
    for (int i = 0; i < HILBERT_TAPS; i++) hb->line[i] = 0.0;
    hb->pos = 0;
}

void hilbert_run(hilbert_t *hb, double x, double *re, double *im) {
    int M = (HILBERT_TAPS - 1) / 2;
    hb->line[hb->pos] = x;
    double acc = 0.0;
    int idx = hb->pos;
    for (int k = 0; k < HILBERT_TAPS; k++) {
        acc += hb->h[k] * hb->line[idx];
        idx = (idx - 1 + HILBERT_TAPS) % HILBERT_TAPS;
    }
    /* Real path delayed by M to align with the quadrature path. */
    int mid = (hb->pos - M + HILBERT_TAPS) % HILBERT_TAPS;
    *re = hb->line[mid];
    *im = acc;
    hb->pos = (hb->pos + 1) % HILBERT_TAPS;
}
