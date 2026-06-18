#ifndef LINE_SIM_RESAMPLE_H
#define LINE_SIM_RESAMPLE_H

/* ── Integer-ratio polyphase resampler: external 8 kHz / 16 kHz <-> 48 kHz ──
 * The line simulator runs its impairments at a fixed 48 kHz internal rate.
 * 48000/8000 = 6 and 48000/16000 = 3 are exact integers, so the up- and
 * down-samplers are clean L-phase polyphase FIRs sharing one windowed-sinc
 * prototype low-pass (designed at init). Linear-phase => constant group delay,
 * which keeps the relay's sample counts exact (N external in -> N external out
 * absent clock slip). */

#define RS_K        36                  /* prototype taps per polyphase branch */
#define RS_L_MAX     6                  /* max upsample factor (8 kHz case)    */
#define RS_PROTO_MAX (RS_K * RS_L_MAX)

typedef struct {
    int    L;                           /* 6 (8 kHz) or 3 (16 kHz)             */
    /* Interpolator: prototype scaled to passband gain L, stored polyphase. */
    double up[RS_L_MAX][RS_K];
    double up_hist[RS_K];               /* external-rate input history         */
    int    up_pos;
    /* Decimator: prototype scaled to passband gain 1, full length. */
    double down[RS_PROTO_MAX];
    int    down_len;
    double down_hist[RS_PROTO_MAX];     /* internal-rate (48 kHz) history       */
    int    down_pos;
    int    down_phase;                  /* counts 0..L-1; emit when it wraps    */
} resamp_t;

/* ext_rate must be 8000 or 16000. Returns 0 on success, -1 on bad rate. */
int  resamp_init(resamp_t *r, int ext_rate);
void resamp_reset(resamp_t *r);

/* Push one external-rate sample; writes exactly L internal (48 kHz) samples
 * to out[] and returns L. */
int  resamp_up(resamp_t *r, double x, double *out);

/* Push one internal (48 kHz) sample. Returns 1 and writes *out on the L-th
 * call (one external-rate output), else returns 0. */
int  resamp_down(resamp_t *r, double x, double *out);

#endif /* LINE_SIM_RESAMPLE_H */
