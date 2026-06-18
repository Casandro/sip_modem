/* ITU-T G.722 sub-band ADPCM codec, 64 kbit/s (mode 1).
 * Own implementation of the G.722 algorithm; validated bit-exactly against
 * the ITU G.722 test vectors. Public domain. */

#include "g722.h"
#include <string.h>

/* ── 16-bit "basic operator" arithmetic (saturating) ──────────────── */

static int sat16(long v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int)v;
}
static int add16(int a, int b)  { return sat16((long)a + b); }
static int sub16(int a, int b)  { return sat16((long)a - b); }
static int neg16(int a)         { return sat16(-(long)a); }
static int shl16(int a, int n)  { return sat16((long)a << n); }
/* mult: extract_h(L_mult(a,b)) == sat16((a*b) >> 15) */
static int mult16(int a, int b) { return sat16(((long)a * b) >> 15); }

/* 32-bit saturating add (for the QMF accumulators). */
static int32_t Lsat(long long v) {
    if (v >  2147483647LL) return  2147483647;
    if (v < -2147483648LL) return (int32_t)(-2147483648LL);
    return (int32_t)v;
}

static int limit_pcm(int rl) {            /* G.722 'limit': clamp to ±2^14 */
    if (rl >  16383) return  16383;
    if (rl < -16384) return -16384;
    return rl;
}

/* ── Tables (ITU-T G.722) ─────────────────────────────────────────── */

/* QMF coefficients, pre-doubled (as in the reference coef_qmf). */
static const int qmf[24] = {
    3*2, -11*2, -11*2, 53*2, 12*2, -156*2, 32*2, 362*2,
    -210*2, -805*2, 951*2, 3876*2, 3876*2, 951*2, -805*2, -210*2,
    362*2, 32*2, -156*2, 12*2, 53*2, -11*2, -11*2, 3*2
};

static const int q6[31] = {
    0, 35, 72, 110, 150, 190, 233, 276, 323, 370, 422, 473, 530, 587, 650,
    714, 786, 858, 940, 1023, 1121, 1219, 1339, 1458, 1612, 1765, 1980,
    2195, 2557, 2919, 3200
};
static const int misil[2][32] = {
    {0x00,0x3F,0x3E,0x1F,0x1E,0x1D,0x1C,0x1B,0x1A,0x19,0x18,0x17,0x16,0x15,
     0x14,0x13,0x12,0x11,0x10,0x0F,0x0E,0x0D,0x0C,0x0B,0x0A,0x09,0x08,0x07,
     0x06,0x05,0x04,0x00},
    {0x00,0x3D,0x3C,0x3B,0x3A,0x39,0x38,0x37,0x36,0x35,0x34,0x33,0x32,0x31,
     0x30,0x2F,0x2E,0x2D,0x2C,0x2B,0x2A,0x29,0x28,0x27,0x26,0x25,0x24,0x23,
     0x22,0x21,0x20,0x00}
};
static const int misih[2][3] = { {0,1,0}, {0,3,2} };

static const int ril4[16] = {0,7,6,5,4,3,2,1,7,6,5,4,3,2,1,0};
static const int risil[16] = {0,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0};
static const int oq4[8] = {0,150,323,530,786,1121,1612,2557};

static const int ril6[64] = {
    1,1,1,1,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,
    9,8,7,6,5,4,3,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,
    11,10,9,8,7,6,5,4,3,2,1,2,1
};
static const int risi6[64] = {
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,-1
};
static const int oq6[31] = {
    0,17,54,91,130,170,211,254,300,347,396,447,501,558,618,682,750,822,899,
    982,1072,1170,1279,1399,1535,1689,1873,2088,2376,2738,3101
};

static const int ih2[4] = {2,1,2,1};
static const int sih[4] = {-1,-1,0,0};
static const int oq2[3] = {0,202,926};
static const int wh[3]  = {0,-214,798};
static const int wl[8]  = {-60,-30,58,172,334,538,1198,3042};

static const int ila[353] = {
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,
    3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,6,6,6,7,7,7,7,7,7,8,8,
    8,8,8,9,9,9,9,10,10,10,10,11,11,11,11,12,12,12,13,13,13,13,14,14,15,15,
    15,16,16,16,17,17,18,18,18,19,19,20,20,21,21,22,22,23,23,24,24,25,25,26,
    27,27,28,28,29,30,31,31,32,33,33,34,35,36,37,37,38,39,40,41,42,43,44,45,
    46,47,48,49,50,51,52,54,55,56,57,58,60,61,63,64,65,67,68,70,71,73,75,76,
    78,80,82,83,85,87,89,91,93,95,97,99,102,104,106,109,111,113,116,118,121,
    124,127,129,132,135,138,141,144,147,151,154,157,161,165,168,172,176,180,
    184,188,192,196,200,205,209,214,219,223,228,233,238,244,249,255,260,266,
    272,278,284,290,296,303,310,316,323,331,338,345,353,361,369,377,385,393,
    402,411,420,429,439,448,458,468,478,489,500,511,522,533,545,557,569,582,
    594,607,621,634,648,663,677,692,707,723,739,755,771,788,806,823,841,860,
    879,898,918,938,958,979,1001,1023,1045,1068,1092,1115,1140,1165,1190,
    1216,1243,1270,1298,1327,1356,1386,1416,1447,1479,1511,1544,1578,1613,
    1648,1684,1721,1759,1797,1837,1877,1918,1960,2003,2047,2092,2138,2185,
    2232,2281,2331,2382,2434,2488,2542,2598,2655,2713,2773,2833,2895,2959,
    3024,3090,3157,3227,3297,3370,3443,3519,3596,3675,3755,3837,3921,4007,
    4095
};

/* ── Quantizers ───────────────────────────────────────────────────── */

static int quantl(int el, int det) {
    int sil = (el < 0) ? -1 : 0;
    int wd  = (sil == 0) ? el : (32767 - (el & 32767));
    int mil = 0;
    int val = mult16(shl16(q6[0], 3), det);
    while (val <= wd) {
        if (mil == 30) break;
        mil++;
        val = mult16(shl16(q6[mil], 3), det);
    }
    return misil[sil + 1][mil];
}

static int quanth(int eh, int det) {
    int sih2 = (eh < 0) ? -1 : 0;
    int wd   = (sih2 == 0) ? eh : (32767 - (eh & 32767));
    int mih  = (wd >= mult16(shl16(564, 3), det)) ? 2 : 1;
    return misih[sih2 + 1][mih];
}

/* ── Inverse quantizers ───────────────────────────────────────────── */

static int invqal(int il, int det) {            /* 4-bit, predictor feedback */
    int ril = il >> 2;
    int wd1 = shl16(oq4[ril4[ril]], 3);
    int wd2 = (risil[ril] == 0) ? wd1 : neg16(wd1);
    return mult16(det, wd2);
}
static int invqbl(int ilr, int det) {           /* 6-bit (mode 1), output */
    int wd1 = shl16(oq6[ril6[ilr]], 3);
    int wd2 = (risi6[ilr] == 0) ? wd1 : neg16(wd1);
    return mult16(det, wd2);
}
static int invqah(int ih, int det) {
    int wd1 = shl16(oq2[ih2[ih]], 3);
    int wd2 = (sih[ih] == 0) ? wd1 : neg16(wd1);
    return mult16(wd2, det);
}

/* ── Scale-factor adaptation ──────────────────────────────────────── */

static int logscl(int il, int nb) {
    int nbpl = add16(mult16(nb, 32512), wl[ril4[il >> 2]]);
    if (nbpl < 0) nbpl = 0;
    if (nbpl > 18432) nbpl = 18432;
    return nbpl;
}
static int logsch(int ih, int nb) {
    int nbph = add16(mult16(nb, 32512), wh[ih2[ih]]);
    if (nbph < 0) nbph = 0;
    if (nbph > 22528) nbph = 22528;
    return nbph;
}
static int scalel(int nb) {
    int wd2 = ((nb >> 6) & 511) + 64;
    return shl16(add16(ila[wd2], 1), 2);
}
static int scaleh(int nb) {
    int wd = (nb >> 6) & 511;
    return shl16(add16(ila[wd], 1), 2);
}

/* ── Adaptive predictor (shared by both sub-bands) ────────────────── */

static int filtez(const int *d, const int *b) {
    int sz = 0;
    for (int i = 6; i > 0; i--)
        sz = add16(sz, mult16(add16(d[i], d[i]), b[i]));
    return sz;
}
static int filtep(int *r, const int *a) {
    r[2] = r[1];
    r[1] = r[0];
    return add16(mult16(a[1], add16(r[1], r[1])),
                 mult16(a[2], add16(r[2], r[2])));
}
static void upzero(int *d, int *b) {
    int wd1 = (d[0] == 0) ? 0 : 128;
    int sg0 = (d[0] < 0) ? -1 : 0;
    for (int i = 6; i > 0; i--) {
        int sgi = (d[i] < 0) ? -1 : 0;
        int wd2 = (sg0 == sgi) ? wd1 : -wd1;
        b[i] = add16(wd2, mult16(b[i], 32640));
        d[i] = d[i - 1];
    }
}
static void uppol2(int *a, const int *p) {
    int sg0 = (p[0] < 0) ? -1 : 0;
    int sg1 = (p[1] < 0) ? -1 : 0;
    int sg2 = (p[2] < 0) ? -1 : 0;
    int wd1 = shl16(a[1], 2);
    int wd2 = (sg0 == sg1) ? sub16(0, wd1) : wd1;
    wd2 = wd2 >> 7;
    int wd3 = (sg0 == sg2) ? 128 : -128;
    int apl2 = add16(add16(wd2, wd3), mult16(a[2], 32512));
    if (apl2 >  12288) apl2 =  12288;
    if (apl2 < -12288) apl2 = -12288;
    a[2] = apl2;
}
static void uppol1(int *a, int *p) {
    int sg0 = (p[0] < 0) ? -1 : 0;
    int sg1 = (p[1] < 0) ? -1 : 0;
    int wd1 = (sg0 == sg1) ? 192 : -192;
    int apl1 = add16(wd1, mult16(a[1], 32640));
    int wd3 = sub16(15360, a[2]);
    if (apl1 > wd3) apl1 = wd3;
    else if (apl1 < -wd3) apl1 = neg16(wd3);
    p[2] = p[1];
    p[1] = p[0];
    a[1] = apl1;
}

/* Common predictor update after d[0], p[0], r[0] are set. */
static void predict(g722_band_t *bnd) {
    upzero(bnd->d, bnd->b);
    uppol2(bnd->a, bnd->p);
    uppol1(bnd->a, bnd->p);
    bnd->sz = filtez(bnd->d, bnd->b);
    bnd->sp = filtep(bnd->r, bnd->a);
    bnd->s  = add16(bnd->sp, bnd->sz);
}

/* ── QMF ──────────────────────────────────────────────────────────── */

static void qmf_analysis(int *dl, int xin0, int xin1, int *xl, int *xh) {
    dl[1] = xin1;
    dl[0] = xin0;
    long long ea = 0, eb = 0;
    for (int j = 0; j < 12; j++) {
        ea = Lsat(ea + (long long)qmf[2*j]   * dl[2*j]);
        eb = Lsat(eb + (long long)qmf[2*j+1] * dl[2*j+1]);
    }
    for (int j = 23; j >= 2; j--) dl[j] = dl[j - 2];
    int32_t lo = Lsat(Lsat(ea + eb) * 2LL);   /* (accuma+accumb)*2 */
    int32_t hi = Lsat(Lsat(ea - eb) * 2LL);   /* (accuma-accumb)*2 */
    *xl = limit_pcm((int)(lo >> 16));
    *xh = limit_pcm((int)(hi >> 16));
}

static void qmf_synthesis(int *dl, int rl, int rh, int *xout1, int *xout2) {
    dl[1] = add16(rl, rh);
    dl[0] = sub16(rl, rh);
    long long ea = 0, eb = 0;
    for (int j = 0; j < 12; j++) {
        ea = Lsat(ea + (long long)qmf[2*j]   * dl[2*j]);
        eb = Lsat(eb + (long long)qmf[2*j+1] * dl[2*j+1]);
    }
    for (int j = 23; j >= 2; j--) dl[j] = dl[j - 2];
    int32_t lo = Lsat(ea << 4);
    int32_t hi = Lsat(eb << 4);
    *xout1 = (int)(lo >> 16);   /* extract_h */
    *xout2 = (int)(hi >> 16);
}

/* ── Public API ───────────────────────────────────────────────────── */

static void band_reset(g722_band_t *b, int det) {
    memset(b, 0, sizeof(*b));
    b->det = det;
}
void g722_enc_init(g722_enc_t *s) {
    memset(s, 0, sizeof(*s));
    band_reset(&s->band[0], 32);
    band_reset(&s->band[1], 8);
}
void g722_dec_init(g722_dec_t *s) {
    memset(s, 0, sizeof(*s));
    band_reset(&s->band[0], 32);
    band_reset(&s->band[1], 8);
}

int g722_encode(g722_enc_t *s, const int16_t *pcm, int nsamp, uint8_t *out) {
    g722_band_t *lo = &s->band[0], *hi = &s->band[1];
    int n = 0;
    for (int i = 0; i + 1 < nsamp; i += 2) {
        int xl, xh;
        qmf_analysis(s->qmf, pcm[i + 1], pcm[i], &xl, &xh);

        /* lower sub-band */
        int el = sub16(xl, lo->s);
        int il = quantl(el, lo->det);
        lo->d[0] = invqal(il, lo->det);
        lo->nb = logscl(il, lo->nb);
        lo->det = scalel(lo->nb);
        lo->p[0] = add16(lo->d[0], lo->sz);
        lo->r[0] = add16(lo->s, lo->d[0]);
        predict(lo);

        /* higher sub-band */
        int eh = sub16(xh, hi->s);
        int ih = quanth(eh, hi->det);
        hi->d[0] = invqah(ih, hi->det);
        hi->nb = logsch(ih, hi->nb);
        hi->det = scaleh(hi->nb);
        hi->p[0] = add16(hi->d[0], hi->sz);
        hi->r[0] = add16(hi->s, hi->d[0]);
        predict(hi);

        out[n++] = (uint8_t)(((ih << 6) | il) & 0xFF);
    }
    return n;
}

int g722_decode(g722_dec_t *s, const uint8_t *in, int noct, int16_t *pcm) {
    g722_band_t *lo = &s->band[0], *hi = &s->band[1];
    int n = 0;
    for (int i = 0; i < noct; i++) {
        int il = in[i] & 0x3F;
        int ih = (in[i] >> 6) & 0x03;

        /* lower sub-band */
        int dl = invqbl(il, lo->det);
        int rl = limit_pcm(add16(lo->s, dl));
        lo->d[0] = invqal(il, lo->det);
        lo->nb = logscl(il, lo->nb);
        lo->det = scalel(lo->nb);
        lo->p[0] = add16(lo->d[0], lo->sz);
        lo->r[0] = add16(lo->s, lo->d[0]);
        predict(lo);

        /* higher sub-band */
        hi->d[0] = invqah(ih, hi->det);
        hi->nb = logsch(ih, hi->nb);
        hi->det = scaleh(hi->nb);
        hi->p[0] = add16(hi->d[0], hi->sz);
        hi->r[0] = add16(hi->s, hi->d[0]);
        predict(hi);
        int rh = limit_pcm(hi->r[0]);

        int x1, x2;
        qmf_synthesis(s->qmf, rl, rh, &x1, &x2);
        pcm[n++] = (int16_t)x1;
        pcm[n++] = (int16_t)x2;
    }
    return n;
}
