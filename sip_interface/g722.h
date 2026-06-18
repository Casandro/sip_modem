#ifndef G722_H
#define G722_H

#include <stdint.h>

/* ── ITU-T G.722 sub-band ADPCM codec (64 kbit/s, "mode 1") ───────────
 *
 * 16 kHz / 16-bit linear PCM <-> 64 kbit/s octet stream (8 bits per octet,
 * one octet per two input samples). Encoder and decoder hold independent
 * adaptive state; create one of each per direction per call.
 *
 * Own implementation of the ITU-T G.722 algorithm (Rec. G.722 / G.191 STL),
 * using integer "basic-operator" semantics (16-bit saturating arithmetic).
 * Validated bit-exactly against the ITU G.722 test vectors
 * (inpsp.bin -> codsp.cod and codsp.cod -> outsp1.bin). Public domain. */

/* Per sub-band adaptive predictor + quantizer-scale state. */
typedef struct {
    int a[3];   /* pole coefficients a[1], a[2] */
    int b[7];   /* zero coefficients b[1..6] */
    int d[7];   /* quantized-difference delay d[0..6] */
    int p[3];   /* partial-reconstructed-signal delay p[0..2] */
    int r[3];   /* reconstructed-signal delay r[0..2] */
    int det;    /* quantizer scale factor */
    int nb;     /* logarithmic quantizer scale factor */
    int s;      /* predicted signal (pole + zero) */
    int sp;     /* pole section output */
    int sz;     /* zero section output */
} g722_band_t;

typedef struct {
    g722_band_t band[2];   /* [0] = lower sub-band, [1] = higher sub-band */
    int qmf[24];           /* analysis QMF delay line */
} g722_enc_t;

typedef struct {
    g722_band_t band[2];
    int qmf[24];           /* synthesis QMF delay line */
} g722_dec_t;

void g722_enc_init(g722_enc_t *s);
void g722_dec_init(g722_dec_t *s);

/* Encode `nsamp` (even) 16 kHz samples into nsamp/2 octets; returns octets. */
int  g722_encode(g722_enc_t *s, const int16_t *pcm, int nsamp, uint8_t *out);

/* Decode `noct` octets into 2*noct 16 kHz samples (mode 1); returns samples. */
int  g722_decode(g722_dec_t *s, const uint8_t *in, int noct, int16_t *pcm);

#endif /* G722_H */
