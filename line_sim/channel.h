#ifndef LINE_SIM_CHANNEL_H
#define LINE_SIM_CHANNEL_H

/* ── One-direction impairment pipeline ─────────────────────────────────────
 * A chan_t carries one direction of audio (A->B or B->A). chan_process()
 * takes external-rate (8/16 kHz) s16le samples, upsamples to 48 kHz, applies
 * the configured line impairments, downsamples back, applies the clock-slip
 * (random sample skip/duplicate, bounded to ±SLIP_MAX), and pushes the result
 * into an output FIFO the relay drains in fixed blocks.
 *
 * Chain order (physical): gain -> frequency response -> group-delay (phase)
 * distortion -> non-linear distortion -> frequency offset + phase jitter   (all
 * at 48 kHz) -> downsample -> optional G.711 A-law round-trip -> additive
 * noise -> clip -> clock-slip. */

#include <stdint.h>
#include "resample.h"
#include "filters.h"

#define LS_MAX_BUMPS  4
#define LS_MAX_FR     (1 + LS_MAX_BUMPS)   /* high-shelf + peaking sections */
#define LS_MAX_FIR    257
#define SLIP_MAX      10                   /* hard bound on cumulative slip */

/* Sample FIFO (the relay's elastic store; absorbs ±1 slip transients). */
#define FIFO_CAP 512
typedef struct {
    int16_t buf[FIFO_CAP];
    int     head, tail, count;
} fifo_t;

void fifo_init(fifo_t *f);
void fifo_push(fifo_t *f, int16_t v);
int  fifo_pop(fifo_t *f, int16_t *out, int n);   /* returns # popped (<=n) */

/* Configuration for one direction. Zeroed = clean passthrough. */
typedef struct {
    double gain_db;                      /* flat gain/attenuation            */
    double freq_tilt_db;                 /* high-shelf: gain@3400 - gain@300 */
    struct { double f0, q, gain_db; } bump[LS_MAX_BUMPS];
    int    nbumps;
    const char *fir_path;                /* optional arbitrary-FIR mask file */
    double gd_us;                        /* peak group-delay distortion (µs) */
    int    has_thd;  double thd_db;      /* signal-to-harmonic-distortion    */
    int    has_snr;  double snr_db;      /* signal-to-noise ratio            */
    int    noise_cmsg;                   /* C-message-weight the noise        */
    double offset_hz;                    /* carrier frequency shift           */
    double jitter_deg, jitter_rate_hz;   /* phase jitter peak deg @ rate      */
    int    alaw;                         /* G.711 A-law round-trip            */
    double slip_prob;                    /* per-output-sample slip probability */
} chan_cfg_t;

typedef struct {
    int        ext_rate;
    resamp_t   rs;

    /* Frequency response. */
    biquad_t   fr[LS_MAX_FR];
    int        n_fr;
    double     fir[LS_MAX_FIR];
    double     fir_hist[LS_MAX_FIR];
    int        fir_len, fir_pos;

    /* Group-delay (phase) distortion. */
    biquad_t   gd[2];
    int        n_gd;

    /* Non-linear distortion (memoryless tanh, calibrated to thd_db). */
    int        nl_on;
    double     nl_g;                     /* tanh gain                         */
    double     nl_norm;                  /* 1/tanh(g) normalization            */

    /* Frequency offset + phase jitter (share the Hilbert analytic front end). */
    int        oj_on;
    hilbert_t  hb;
    double     off_phase;                /* offset accumulator (rad)          */
    double     off_step;                 /* 2π·offset_hz / 48000              */
    double     jit_peak;                 /* deg -> rad                        */
    double     jit_step;                 /* 2π·rate / 48000                   */
    double     jit_phase;

    /* Additive noise (applied at external rate, referenced to signal power). */
    int        noise_on;
    double     snr_lin;                  /* 10^(snr_db/10)                    */
    double     ps;                       /* running signal-power estimate     */
    int        noise_cmsg;
    biquad_t   cmsg;                     /* approximate C-message weighting   */
    double     cmsg_scale;               /* renormalize weighted-noise power  */
    uint32_t   nrng;                     /* noise PRNG                        */

    double     gain_lin;
    int        alaw;

    /* Clock slip. */
    double     slip_prob;
    int        slip_accum;               /* output_count - input_count, |·|≤10 */
    uint32_t   srng;                     /* slip PRNG                         */

    /* Diagnostics. */
    long long  in_count, out_count;
    int        slip_min, slip_max;
} chan_t;

/* Build the pipeline from cfg. seed varies the noise/slip PRNGs per direction.
 * Returns 0 on success, -1 on a bad cfg (e.g. unreadable FIR file). */
int  chan_init(chan_t *c, const chan_cfg_t *cfg, int ext_rate, uint32_t seed);
void chan_reset(chan_t *c);

/* Process n external-rate samples; push the impaired output into `out`. */
void chan_process(chan_t *c, const int16_t *in, int n, fifo_t *out);

#endif /* LINE_SIM_CHANNEL_H */
