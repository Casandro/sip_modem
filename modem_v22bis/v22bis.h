#ifndef V22BIS_H
#define V22BIS_H

#include <stdint.h>

/* ── ITU-T V.22bis modem core ──────────────────────────────────────────
 *
 * Full-duplex voiceband modem: 2400 bit/s (16-QAM) with 1200 bit/s (QPSK)
 * fallback, 600 baud, band-split with a 1200 Hz (calling) / 2400 Hz
 * (answering) carrier. Sampled at 8 kHz, int16 PCM — the same audio format
 * the rest of the suite uses, so it bridges over sip_interface unchanged.
 *
 * The API mirrors spandsp's v22bis (block-based tx/rx + get_bit/put_bit
 * callbacks) on purpose: it makes the modem a drop-in for the same harness
 * shape as modem_fsk, and lets the test harness drive this core and spandsp's
 * reference modem through one code path for interop verification.
 *
 * Design parameters are locked to spandsp 0.0.6 (the installed reference) so
 * the two interoperate at the signal level. The one deliberate divergence is
 * the data scrambler: by default this core uses the ITU per-direction
 * polynomials (calling 1+x^-14+x^-17, answering 1+x^-18+x^-23). spandsp uses
 * 1+x^-14+x^-17 for both directions; pass spandsp_compat=1 to match it for a
 * full data-BER interop test. */

#define V22_SAMPLE_RATE   8000
#define V22_BAUD          600

/* Filter / equalizer sizing (mirrors spandsp private/v22bis.h). */
#define V22_TX_FILTER_STEPS   9     /* TX RRC polyphase taps per coeff set   */
#define V22_RX_FILTER_STEPS   27    /* RX RRC/bandpass filter taps           */
#define V22_EQ_LEN            7     /* equalizer half-length; full = 2*N+1   */
#define V22_EQ_MASK          15     /* (1<<ceil(log2(2*EQ_LEN+1))) - 1        */

/* RX matched-filter geometry (canonical here; v22bis_rrc.h must agree, which a
 * _Static_assert in v22bis_rx.c checks). Regenerate the coeffs after changing. */
#define V22_RX_MF_TAPS      107     /* real RRC matched filter @8 kHz, ±4 sym */
#define V22_RX_MF_DELAY      53     /* group delay (taps-1)/2                  */

/* Guard tone (answering modem only). */
enum { V22_GUARD_NONE = 0, V22_GUARD_550 = 1, V22_GUARD_1800 = 2 };

/* Convert a frequency in Hz to a Q32 per-sample DDS phase increment. */
#define V22_DDS_RATE(hz)  ((int32_t)((hz) * 4294967296.0 / V22_SAMPLE_RATE))

typedef struct { float re, im; } v22_cf;

/* Bit I/O callbacks. get_bit returns the next bit to transmit (0/1), or 1 when
 * the data source is idle (V.22bis transmits scrambled ones while idle).
 * put_bit consumes one recovered data bit. */
typedef int  (*v22_get_bit_t)(void *user);
typedef void (*v22_put_bit_t)(void *user, int bit);

/* ── Modem state ──────────────────────────────────────────────────────── */

typedef struct v22bis_s {
    int  max_bit_rate;          /* 1200 or 2400 (ceiling; may negotiate down) */
    int  negotiated_bit_rate;   /* 0 until trained, then 1200 or 2400         */
    int  calling_party;         /* 1 = calling (originate), 0 = answering     */
    int  guard;                 /* V22_GUARD_*                                */
    int  spandsp_compat;        /* 1 = 14/17 scrambler both ways (see header) */
    float amplitude;            /* peak output scale applied to unit symbols  */

    v22_get_bit_t get_bit;  void *get_user;
    v22_put_bit_t put_bit;  void *put_user;

    /* Transmit section. */
    struct {
        uint32_t scramble_reg;
        int      scrambler_ones;        /* consecutive-ones guard (resets @64) */
        int      tap1, tap2;            /* scrambler shift taps for this side  */

        int      training;              /* V22_TX_STAGE_*                      */
        int      training_count;        /* symbols spent in the current stage  */

        uint32_t carrier_phase;  int32_t carrier_phase_rate;
        uint32_t guard_phase;    int32_t guard_phase_rate;
        float    guard_level;
        float    gain;                  /* output scale for unit-ish symbols   */

        v22_cf   rrc[2 * V22_TX_FILTER_STEPS];  /* pulse-shaper history (I/Q)  */
        int      rrc_step;
        int      baud_phase;            /* polyphase resampler accumulator     */
        int      constellation_state;   /* current quadrant 0..3               */
        int      current_bits;          /* bits being shaped this symbol       */
        int      shutdown;
        void   (*sym_report)(void *user, int idx);  /* diagnostics             */
        void    *sym_user;
    } tx;

    /* Receive section.
     * Front end: NCO carrier demod to baseband -> complex RRC matched filter
     * (full 8 kHz rate) -> Gardner-driven fractional resampler to T/2 symbols
     * -> fractionally-spaced complex LMS equalizer -> slicer -> differential
     * decode + descramble. Carrier tracked by a decision-directed PI loop. */
    struct {
        /* NCO carrier (demodulation to baseband). */
        double   carrier_phase;         /* radians                              */
        double   carrier_phase_rate;    /* radians/sample (nominal -2pi f/fs)   */
        double   carrier_kp, carrier_ki;/* PI loop gains (0 = freeze)           */

        /* AGC / carrier detect. */
        float    agc;
        float    mf_power;              /* running mean-square of |MF output|   */
        float    power;                 /* running mean-square of input         */
        int      acq;                   /* samples since carrier came up        */
        int      agc_warm;              /* >0 while the AGC is (re)calibrating   */
        int      signal_present;

        /* Complex matched-filter delay line (baseband), full rate. */
        v22_cf   mf_buf[V22_RX_MF_TAPS];
        int      mf_pos;

        /* Gardner-driven T/2 resampler over the matched-filter output. */
        v22_cf   y_hist[4];             /* recent MF outputs for interpolation  */
        int      y_count;               /* MF outputs produced so far           */
        double   next_t;                /* input-sample time of next T/2 sample */
        double   half_period;           /* samples per half symbol (tracked)    */
        int      gardner_integrate, gardner_step;
        double   gardner_kp, gardner_ki;   /* timing loop gains (0 = freeze)    */

        /* Fractionally-spaced (T/2) complex LMS equalizer. */
        v22_cf   eq_coeff[2 * V22_EQ_LEN + 1];
        v22_cf   eq_buf[V22_EQ_MASK + 1];
        int      eq_step;
        int      half;                  /* 0/1: which half-baud just arrived    */
        float    eq_delta;

        /* Descrambler. */
        uint32_t scramble_reg;
        int      scrambler_ones;
        int      tap1, tap2;

        /* Differential decode + training. */
        int      training;              /* V22_RX_STAGE_*                       */
        int      training_count;
        int      constellation_state;
        int      last_raw_bits, pattern_repeats;
        int      sixteen_way;           /* 1 = 16-QAM decisions, 0 = QPSK       */
        int      carrier_present;

        /* Optional per-baud constellation report (diagnostics). */
        void   (*qam_report)(void *user, float zr, float zi, int nearest);
        void    *qam_user;
    } rx;
} v22bis_t;

/* ── Lifecycle ────────────────────────────────────────────────────────── */

/* Initialise s in place. max_bit_rate is 1200 or 2400; calling_party selects
 * the originate (1) or answer (0) side; guard is a V22_GUARD_* tone (answering
 * only); spandsp_compat forces the 14/17 scrambler both ways. amplitude is the
 * int16 peak output magnitude. Returns s, or NULL on a bad parameter. */
v22bis_t *v22bis_init(v22bis_t *s, int max_bit_rate, int calling_party,
                      int guard, int spandsp_compat,
                      v22_get_bit_t get_bit, void *get_user,
                      v22_put_bit_t put_bit, void *put_user,
                      int amplitude);

/* Reset the receive section (carrier/AGC/equalizer/timing/training). Called by
 * v22bis_init; exposed for a carrier-loss retrain. */
void v22bis_rx_restart(v22bis_t *s);

/* ── Per-block processing (8 kHz int16) ───────────────────────────────── */

/* Generate len transmit samples into amp. Returns the number produced. */
int v22bis_tx(v22bis_t *s, int16_t amp[], int len);

/* Process len received samples from amp. Returns the number consumed. */
int v22bis_rx(v22bis_t *s, const int16_t amp[], int len);

/* ── Status ───────────────────────────────────────────────────────────── */

/* 0 until trained, then the negotiated rate (1200 or 2400). */
int v22bis_current_bit_rate(const v22bis_t *s);

/* 1 once the receiver has trained and is passing data, else 0. */
int v22bis_rx_trained(const v22bis_t *s);

/* 1 while a received carrier is present (hysteretic), else 0. */
int v22bis_rx_carrier(const v22bis_t *s);

/* ── TX training stages (calling and answering share the enum) ──────────── */
enum {
    V22_TX_STAGE_NORMAL = 0,
    V22_TX_STAGE_INITIAL_SILENCE,     /* calling: silent until RX drives it    */
    V22_TX_STAGE_U11,                 /* unscrambled ones                      */
    V22_TX_STAGE_U0011,               /* S1: alternating 00/11 (2400 signal)   */
    V22_TX_STAGE_S11,                 /* scrambled ones at 1200                */
    V22_TX_STAGE_TIMED_S11,           /* timed scrambled ones at 1200          */
    V22_TX_STAGE_S1111,               /* scrambled ones at 2400                */
    V22_TX_STAGE_PARKED
};

/* ── RX training stages ──────────────────────────────────────────────────*/
enum {
    V22_RX_STAGE_NORMAL = 0,
    V22_RX_STAGE_SYMBOL_ACQUISITION,
    V22_RX_STAGE_UNSCRAMBLED_ONES,             /* calling only                 */
    V22_RX_STAGE_UNSCRAMBLED_ONES_SUSTAINING,  /* calling only                 */
    V22_RX_STAGE_SCRAMBLED_ONES_AT_1200,
    V22_RX_STAGE_SCRAMBLED_ONES_AT_1200_SUSTAINING,
    V22_RX_STAGE_WAIT_FOR_SCRAMBLED_ONES_AT_2400,
    V22_RX_STAGE_PARKED
};

/* Shared 16-point constellation (I/Q in {±1,±3}), indexed
 * (quadrant<<2)|inner2, defined in v22bis_tx.c. */
extern const v22_cf v22_constellation[16];

#endif /* V22BIS_H */
