/* Standalone DSP unit checks for line_sim's channel core — no sockets.
 * Drives known stimuli through one chan_t and asserts the achieved impairment
 * matches the requested value: frequency response, group-delay distortion,
 * non-linear distortion (THD), additive-noise SNR, resampler in-band fidelity,
 * the clock-slip ±10 bound, and the lockstep sample-count invariant.
 * Build/run from line_sim/:  make test  */
#define _GNU_SOURCE
#include "channel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int g_fail = 0;
static void check(const char *name, int ok, const char *detail) {
    printf("  %-34s %s  %s\n", name, ok ? "PASS" : "FAIL", detail);
    if (!ok) g_fail = 1;
}

static int16_t clip16(double v) {
    return v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)lrint(v);
}

/* Run a single tone (freq f, amplitude amp) through a fresh channel and gather
 * `maxn` external-rate output samples. Returns the count gathered. */
static int process_tone(const chan_cfg_t *cfg, int ext, double f, double amp,
                        int16_t *out, int maxn) {
    chan_t c;
    chan_init(&c, cfg, ext, 0x12345u);
    chan_reset(&c);
    fifo_t fifo; fifo_init(&fifo);
    int16_t in[40], tmp[64];
    int produced = 0;
    long n = 0;
    while (produced < maxn) {
        for (int i = 0; i < 40; i++) {
            in[i] = clip16(amp * sin(2.0 * M_PI * f * n / ext));
            n++;
        }
        chan_process(&c, in, 40, &fifo);
        int g;
        while (produced < maxn && (g = fifo_pop(&fifo, tmp, 64)) > 0)
            for (int k = 0; k < g && produced < maxn; k++) out[produced++] = tmp[k];
    }
    return produced;
}

/* Complex DFT bin over [start, start+len) at frequency f. */
static void dft_bin(const int16_t *x, int start, int len, double f, double fs,
                    double *re, double *im) {
    double r = 0, i = 0;
    for (int k = 0; k < len; k++) {
        double a = 2.0 * M_PI * f * (start + k) / fs;
        r += x[start + k] * cos(a);
        i -= x[start + k] * sin(a);
    }
    *re = r; *im = i;
}
static double amp_at(const int16_t *x, int start, int len, double f, double fs) {
    double re, im; dft_bin(x, start, len, f, fs, &re, &im);
    return 2.0 * sqrt(re * re + im * im) / len;
}
static double phase_at(const int16_t *x, int start, int len, double f, double fs) {
    double re, im; dft_bin(x, start, len, f, fs, &re, &im);
    return atan2(im, re);
}

/* ── Tests ─────────────────────────────────────────────────────────────── */

static void test_resampler_fidelity(int ext) {
    chan_cfg_t cfg; memset(&cfg, 0, sizeof(cfg));   /* clean passthrough */
    int N = 8000, A = 8000;
    static int16_t out[8000];
    double freqs[] = {300, 1000, 1800, 2800, 3400};
    double maxripple = 0;
    for (int i = 0; i < 5; i++) {
        process_tone(&cfg, ext, freqs[i], A, out, N);
        double g = amp_at(out, 2000, 4000, freqs[i], ext) / A;
        double db = 20 * log10(g);
        if (fabs(db) > maxripple) maxripple = fabs(db);
    }
    char d[64]; snprintf(d, sizeof(d), "max in-band ripple %.2f dB", maxripple);
    check("resampler in-band ripple", maxripple < 0.6, d);
}

static void test_freq_response(int ext) {
    chan_cfg_t cfg; memset(&cfg, 0, sizeof(cfg));
    cfg.freq_tilt_db = 6.0;
    int N = 8000, A = 6000;
    static int16_t out[8000];
    process_tone(&cfg, ext, 300, A, out, N);
    double lo = 20 * log10(amp_at(out, 2000, 4000, 300, ext) / A);
    process_tone(&cfg, ext, 3400, A, out, N);
    double hi = 20 * log10(amp_at(out, 2000, 4000, 3400, ext) / A);
    double tilt = hi - lo;
    char d[80]; snprintf(d, sizeof(d), "requested 6.0 dB, got %.2f dB", tilt);
    check("freq-tilt accuracy", fabs(tilt - 6.0) < 1.0, d);
}

static void test_thd(int ext) {
    chan_cfg_t cfg; memset(&cfg, 0, sizeof(cfg));
    cfg.has_thd = 1; cfg.thd_db = 30.0;
    int N = 16000;
    double A = 0.30 * 32768.0;                /* match NL_REF_AMP */
    static int16_t out[16000];
    process_tone(&cfg, ext, 300, A, out, N);
    double pf = amp_at(out, 4000, 8000, 300, ext);
    double ph = 0;
    int harm[] = {3, 5, 7, 9, 11};
    for (int i = 0; i < 5; i++) {
        double hz = 300.0 * harm[i];
        if (hz < 0.45 * ext) { double a = amp_at(out, 4000, 8000, hz, ext); ph += a * a; }
    }
    double sdr = 10 * log10(pf * pf / ph);
    char d[80]; snprintf(d, sizeof(d), "requested 30 dB, got %.1f dB", sdr);
    check("THD (signal-to-distortion)", fabs(sdr - 30.0) < 2.0, d);
}

static void test_snr(int ext) {
    chan_cfg_t cfg; memset(&cfg, 0, sizeof(cfg));
    cfg.has_snr = 1; cfg.snr_db = 30.0;
    int N = 24000, A = 8000;
    static int16_t out[24000];
    process_tone(&cfg, ext, 1000, A, out, N);
    int s = 8000, len = 12000;                /* steady-state window */
    double tone = amp_at(out, s, len, 1000, ext);
    double ptone = tone * tone / 2.0;
    double ptot = 0;
    for (int i = s; i < s + len; i++) ptot += (double)out[i] * out[i];
    ptot /= len;
    double pnoise = ptot - ptone;
    if (pnoise < 1) pnoise = 1;
    double snr = 10 * log10(ptone / pnoise);
    char d[80]; snprintf(d, sizeof(d), "requested 30 dB, got %.1f dB", snr);
    check("additive-noise SNR", fabs(snr - 30.0) < 2.0, d);
}

static double measure_gd(const chan_cfg_t *cfg, int ext, double f) {
    /* Group delay via phase finite-difference of two filtered tones. */
    int N = 12000, A = 8000;
    static int16_t out[12000];
    double df = 25.0;
    process_tone(cfg, ext, f - df, A, out, N);
    double p1 = phase_at(out, 4000, 6000, f - df, ext);
    process_tone(cfg, ext, f + df, A, out, N);
    double p2 = phase_at(out, 4000, 6000, f + df, ext);
    double dphi = p2 - p1;
    while (dphi >  M_PI) dphi -= 2 * M_PI;
    while (dphi < -M_PI) dphi += 2 * M_PI;
    return -dphi / (2.0 * M_PI * 2.0 * df);
}

static void test_group_delay(int ext) {
    chan_cfg_t cfg; memset(&cfg, 0, sizeof(cfg));
    cfg.gd_us = 1000.0;
    double gd_lo = measure_gd(&cfg, ext, 400);
    double gd_mid = measure_gd(&cfg, ext, 1800);
    double gd_hi = measure_gd(&cfg, ext, 3200);
    double edge = gd_lo > gd_hi ? gd_lo : gd_hi;
    double dist_us = (edge - gd_mid) * 1e6;
    /* Magnitude must stay flat (all-pass). */
    chan_cfg_t mc = cfg;
    int N = 8000, A = 8000;
    static int16_t out[8000];
    double maxdb = 0;
    double tf[] = {400, 1800, 3200};
    for (int i = 0; i < 3; i++) {
        process_tone(&mc, ext, tf[i], A, out, N);
        double db = fabs(20 * log10(amp_at(out, 2000, 4000, tf[i], ext) / A));
        if (db > maxdb) maxdb = db;
    }
    char d[96];
    snprintf(d, sizeof(d), "requested 1000 us, got %.0f us (mag dev %.2f dB)", dist_us, maxdb);
    check("group-delay distortion", dist_us > 600 && dist_us < 1500 && maxdb < 0.6, d);
}

static void test_clock_slip(void) {
    chan_cfg_t cfg; memset(&cfg, 0, sizeof(cfg));
    cfg.slip_prob = 1e-3;
    chan_t c; chan_init(&c, &cfg, 8000, 0xBEEFu); chan_reset(&c);
    fifo_t fifo; fifo_init(&fifo);
    int16_t in[40], tmp[64];
    long total_in = 0, total_out = 0;
    int worst = 0;
    for (int b = 0; b < 250000; b++) {           /* 1e7 samples */
        for (int i = 0; i < 40; i++) in[i] = (int16_t)((b * 40 + i) * 7);
        total_in += 40;
        chan_process(&c, in, 40, &fifo);
        int g;
        while ((g = fifo_pop(&fifo, tmp, 64)) > 0) { total_out += g; if (g < 64) break; }
        if (abs(c.slip_accum) > worst) worst = abs(c.slip_accum);
    }
    long net = total_out - total_in;             /* = slip_accum + fifo residue */
    char d[96];
    snprintf(d, sizeof(d), "|slip| max %d, range [%d,%d], net %+ld",
             worst, c.slip_min, c.slip_max, net);
    check("clock-slip bound (|accum|<=10)",
          worst <= SLIP_MAX && c.slip_min >= -SLIP_MAX && c.slip_max <= SLIP_MAX, d);
}

static void test_lockstep_count(int ext) {
    chan_cfg_t cfg; memset(&cfg, 0, sizeof(cfg));  /* no slip */
    chan_t c; chan_init(&c, &cfg, ext, 1u); chan_reset(&c);
    fifo_t fifo; fifo_init(&fifo);
    int16_t in[40], tmp[64];
    int ok = 1;
    for (int b = 0; b < 1000; b++) {
        for (int i = 0; i < 40; i++) in[i] = (int16_t)(1000 * sin(0.03 * (b * 40 + i)));
        int before = fifo.count;
        chan_process(&c, in, 40, &fifo);
        if (fifo.count - before != 40) ok = 0;
        int g; while ((g = fifo_pop(&fifo, tmp, 64)) > 0) if (g < 64) break;
    }
    char d[64]; snprintf(d, sizeof(d), "rate=%d: exactly 40 out per 40 in", ext);
    check("lockstep sample-count", ok, d);
}

int main(void) {
    printf("line_sim DSP unit checks\n");
    printf("-- external rate 8000 Hz --\n");
    test_resampler_fidelity(8000);
    test_freq_response(8000);
    test_thd(8000);
    test_snr(8000);
    test_group_delay(8000);
    test_lockstep_count(8000);
    printf("-- external rate 16000 Hz --\n");
    test_resampler_fidelity(16000);
    test_lockstep_count(16000);
    printf("-- rate-independent --\n");
    test_clock_slip();
    printf("%s\n", g_fail ? "RESULT: FAIL" : "RESULT: PASS");
    return g_fail ? 1 : 0;
}
