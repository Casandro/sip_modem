/* Channel-impairment self-interop: run two modems through a G.711 A-law
 * companding round-trip (what the sip_interface bridge does to the audio) and
 * optional additive noise, to confirm the modem survives the real audio path.
 * Usage: test_channel [1200|2400] [noise_rms] */
#define _GNU_SOURCE
#include "v22bis.h"
#include "../sip_interface/alaw.h"     /* the repo's G.711 A-law (PCMA path) */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define alaw_enc alaw_encode
#define alaw_dec alaw_decode

/* Cheap deterministic Gaussian-ish noise (sum of uniforms), no libm rand. */
static uint32_t nrng = 0xc0ffee;
static double noise_unit(void) {
    double s = 0;
    for (int i = 0; i < 4; i++) { nrng = nrng * 1103515245u + 12345u; s += ((nrng >> 16) & 0xffff) / 65535.0 - 0.5; }
    return s / 2.0;   /* ~ unit-ish */
}
static int16_t clip16(double v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)v; }

static double g_noise = 0;
static void channel(int16_t *buf, int n) {
    for (int i = 0; i < n; i++) {
        double v = alaw_dec(alaw_enc(buf[i]));     /* G.711 round-trip */
        if (g_noise > 0) v += g_noise * noise_unit();
        buf[i] = clip16(v);
    }
}

#define MAXBITS 700000
typedef struct { uint32_t r; int *s; int n, cap; } prbs_t;
static int pb(prbs_t *p){ p->r^=p->r<<13; p->r^=p->r>>17; p->r^=p->r<<5; int b=p->r&1; if(p->cap&&p->n<MAXBITS)p->s[p->n++]=b; return b; }
typedef struct { int *r; int n, cap; } sink_t;
static void sp(sink_t *s, int b){ if(s->cap&&s->n<MAXBITS)s->r[s->n++]=b&1; }
static prbs_t cS={0x1234u}, aS={0x9e37u}; static sink_t cR, aR;
static int cget(void*u){(void)u;return pb(&cS);} static int aget(void*u){(void)u;return pb(&aS);}
static void cput(void*u,int b){(void)u;sp(&aR,b);} static void aput(void*u,int b){(void)u;sp(&cR,b);}

static double ber(const int*rx,int rn,const int*src,int sn){
    int ws=rn/3,win=rn/3; if(win<1500)return 1.0; int best=win+1;
    for(int off=-2000;off<6000;off++){int e=0,c=0;
      for(int i=0;i<win;i++){int si=ws+i+off; if(si<0||si>=sn)continue; c++; if(rx[ws+i]!=src[si]) if(++e>=best)break;}
      if(c>1000&&e<best)best=e;}
    return (double)best/win;
}

int main(int argc, char **argv) {
    alaw_init();
    int rate = argc > 1 ? atoi(argv[1]) : 2400;
    g_noise = argc > 2 ? atof(argv[2]) : 0.0;
    static int cs[MAXBITS], as[MAXBITS], cr[MAXBITS], ar[MAXBITS];
    cS=(prbs_t){0x1234u,cs,0,0}; aS=(prbs_t){0x9e37u,as,0,0};
    cR=(sink_t){cr,0,0}; aR=(sink_t){ar,0,0};

    v22bis_t c, a;
    v22bis_init(&c, rate, 1, V22_GUARD_NONE, 0, cget, NULL, cput, NULL, 12000);
    v22bis_init(&a, rate, 0, V22_GUARD_NONE, 0, aget, NULL, aput, NULL, 12000);
    int16_t cb[40], ab[40]; int au = -1, cu = -1;
    for (int t = 0; t < 8000 * 9; t += 40) {
        v22bis_tx(&c, cb, 40); v22bis_tx(&a, ab, 40);
        channel(cb, 40); channel(ab, 40);          /* G.711 (+noise) both ways */
        v22bis_rx(&a, cb, 40); v22bis_rx(&c, ab, 40);
        if (au < 0 && v22bis_rx_trained(&a)) au = t;
        if (cu < 0 && v22bis_rx_trained(&c)) cu = t;
        if (au >= 0 && cu >= 0 && !cS.cap) cS.cap = aS.cap = cR.cap = aR.cap = 1;
    }
    if (au < 0 || cu < 0) { printf("rate=%d noise=%.0f: handshake incomplete  FAIL\n", rate, g_noise); return 1; }
    double bca = ber(ar, aR.n, cs, cS.n), bac = ber(cr, cR.n, as, aS.n);
    int ok = bca < 1e-3 && bac < 1e-3;
    printf("rate=%d G.711%s: negotiated=%d  BER call->ans=%.6f ans->call=%.6f  %s\n",
           rate, g_noise > 0 ? "+noise" : "", v22bis_current_bit_rate(&a), bca, bac,
           ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
