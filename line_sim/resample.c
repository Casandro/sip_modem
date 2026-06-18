#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "resample.h"
#include <math.h>
#include <string.h>

#define RS_INTERNAL 48000.0

/* Zeroth-order modified Bessel function, for the Kaiser window. */
static double i0(double x) {
    double sum = 1.0, term = 1.0;
    for (int k = 1; k < 40; k++) {
        double t = x / (2.0 * k);
        term *= t * t;
        sum += term;
        if (term < 1e-12 * sum) break;
    }
    return sum;
}

/* Build a windowed-sinc low-pass prototype of `n` taps at 48 kHz with the
 * given cutoff (Hz), normalized so its DC gain (sum of taps) == target_gain. */
static void design_lowpass(double *h, int n, double cutoff_hz, double target_gain) {
    double wc = 2.0 * M_PI * cutoff_hz / RS_INTERNAL;
    double c = (n - 1) / 2.0;
    double beta = 8.0;                  /* ~70 dB stopband */
    double i0b = i0(beta);
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        double m = i - c;
        double s = (fabs(m) < 1e-9) ? wc / M_PI
                                    : sin(wc * m) / (M_PI * m);
        double r = 2.0 * i / (n - 1) - 1.0;             /* -1..1 */
        double win = i0(beta * sqrt(1.0 - r * r)) / i0b; /* Kaiser */
        h[i] = s * win;
        sum += h[i];
    }
    double scale = target_gain / sum;
    for (int i = 0; i < n; i++) h[i] *= scale;
}

int resamp_init(resamp_t *r, int ext_rate) {
    if (ext_rate != 8000 && ext_rate != 16000) return -1;
    memset(r, 0, sizeof(*r));
    r->L = (int)(RS_INTERNAL / ext_rate);   /* 6 or 3 */
    int n = RS_K * r->L;                     /* 72 or 36 */
    r->down_len = n;

    /* Pass the full voiceband. At 8 kHz the band of interest reaches ~3400 Hz,
     * very close to the 4000 Hz Nyquist, but the nearest image/alias of a
     * 3400 Hz component sits at 8000-3400 = 4600 Hz, so a -6 dB cutoff at
     * 4000 Hz with a sharp (long Kaiser) transition passes 3400 Hz flat and
     * still rejects everything from 4600 Hz up. At 16 kHz there is ample room. */
    double cutoff = (ext_rate == 8000) ? 4000.0 : 0.45 * ext_rate;

    /* Interpolator prototype: passband gain L (restores amplitude lost to
     * zero-stuffing), decomposed into L polyphase branches. */
    double proto_up[RS_PROTO_MAX];
    design_lowpass(proto_up, n, cutoff, (double)r->L);
    for (int p = 0; p < r->L; p++)
        for (int k = 0; k < RS_K; k++)
            r->up[p][k] = proto_up[k * r->L + p];

    /* Decimator prototype: passband gain 1 (anti-alias before dropping). */
    design_lowpass(r->down, n, cutoff, 1.0);
    return 0;
}

void resamp_reset(resamp_t *r) {
    memset(r->up_hist, 0, sizeof(r->up_hist));
    memset(r->down_hist, 0, sizeof(r->down_hist));
    r->up_pos = 0;
    r->down_pos = 0;
    r->down_phase = 0;
}

int resamp_up(resamp_t *r, double x, double *out) {
    /* Push newest input into the circular history. */
    r->up_pos = (r->up_pos + 1) % RS_K;
    r->up_hist[r->up_pos] = x;
    for (int p = 0; p < r->L; p++) {
        double acc = 0.0;
        int idx = r->up_pos;
        for (int k = 0; k < RS_K; k++) {
            acc += r->up[p][k] * r->up_hist[idx];
            idx = (idx - 1 + RS_K) % RS_K;
        }
        out[p] = acc;
    }
    return r->L;
}

int resamp_down(resamp_t *r, double x, double *out) {
    r->down_pos = (r->down_pos + 1) % r->down_len;
    r->down_hist[r->down_pos] = x;
    if (++r->down_phase < r->L) return 0;
    r->down_phase = 0;
    double acc = 0.0;
    int idx = r->down_pos;
    for (int k = 0; k < r->down_len; k++) {
        acc += r->down[k] * r->down_hist[idx];
        idx = (idx - 1 + r->down_len) % r->down_len;
    }
    *out = acc;
    return 1;
}
