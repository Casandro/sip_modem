#ifndef LINE_SIM_FILTERS_H
#define LINE_SIM_FILTERS_H

/* ── Filter primitives for the line simulator ──────────────────────────────
 * RBJ biquads (peaking / high-shelf) for the configurable frequency-response
 * (attenuation-distortion) stage, 2nd-order all-pass sections for the
 * group-delay (phase) distortion equalizer, and a windowed Hilbert FIR used
 * to form the analytic signal for the frequency-offset / phase-jitter stage.
 * All design happens once at init; the per-sample paths are plain difference
 * equations. Double precision throughout — these run at 48 kHz on one stream
 * and accuracy matters more than the cycles. */

#include <stddef.h>

/* Transposed-direct-form-II biquad, a0 normalized to 1. */
typedef struct {
    double b0, b1, b2, a1, a2;
    double z1, z2;
} biquad_t;

void   biquad_reset(biquad_t *f);
double biquad_run(biquad_t *f, double x);
/* Magnitude response |H(e^jw)| at frequency hz (sample rate fs). */
double biquad_mag(const biquad_t *f, double hz, double fs);

/* RBJ cookbook designs (gain_db can be negative). Q sets bandwidth. */
void biquad_peaking(biquad_t *f, double hz, double q, double gain_db, double fs);
void biquad_high_shelf(biquad_t *f, double hz, double gain_db, double fs);

/* A 2nd-order all-pass section: unity magnitude, a group-delay bump peaking
 * near `hz` whose height grows with the pole radius r (0 < r < 1). */
void biquad_allpass2(biquad_t *f, double hz, double r, double fs);

/* Group delay (seconds) of a cascade of `n` sections at frequency hz. */
double biquad_chain_group_delay(const biquad_t *secs, int n, double hz, double fs);

/* ── Hilbert transformer (odd length, type-III FIR) ──────────────────────── */
#define HILBERT_TAPS 31
typedef struct {
    double h[HILBERT_TAPS];     /* quadrature taps (0 for even offsets)      */
    double line[HILBERT_TAPS];  /* input delay line                          */
    int    pos;                 /* write cursor                              */
} hilbert_t;

void hilbert_init(hilbert_t *hb);
void hilbert_reset(hilbert_t *hb);
/* Push x, return the analytic pair: *re is x delayed by the group delay
 * (HILBERT_TAPS/2 samples), *im is the Hilbert transform. */
void hilbert_run(hilbert_t *hb, double x, double *re, double *im);

#endif /* LINE_SIM_FILTERS_H */
