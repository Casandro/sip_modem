/* Full-duplex handshake self-interop: a calling and an answering modem run
 * against each other (each TX feeds the other's RX) with NO hand-orchestration.
 * They automatically train through the V.22bis startup sequence, negotiate the
 * rate, reach NORMAL, and exchange data. Both sides send an independent PRBS;
 * each receiver recovers the other's, so BOTH directions are checked.
 * Usage: test_handshake [1200|2400] */
#define _GNU_SOURCE
#include "v22bis.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define MAXBITS 700000

typedef struct { uint32_t prbs; int src[MAXBITS]; int sn; int cap; } txside_t;
typedef struct { int rx[MAXBITS]; int rn; int cap; } rxside_t;

static int gen(txside_t *t) {
    t->prbs ^= t->prbs << 13; t->prbs ^= t->prbs >> 17; t->prbs ^= t->prbs << 5;
    int b = (int)(t->prbs & 1u);
    if (t->cap && t->sn < MAXBITS) t->src[t->sn++] = b;
    return b;
}
static txside_t ctx_tx = { 0x1234u, {0}, 0, 0 };
static txside_t atx_tx = { 0x9e3779b9u, {0}, 0, 0 };
static rxside_t crx = {{0}, 0, 0}, arx = {{0}, 0, 0};

static int call_get(void *u) { (void)u; return gen(&ctx_tx); }
static int ans_get(void *u)  { (void)u; return gen(&atx_tx); }
static void call_put(void *u, int b) { (void)u; if (crx.cap && crx.rn < MAXBITS) crx.rx[crx.rn++] = b & 1; }
static void ans_put(void *u, int b)  { (void)u; if (arx.cap && arx.rn < MAXBITS) arx.rx[arx.rn++] = b & 1; }

/* Align recovered bits to a source stream and return the BER over a window. */
static double ber(const int *rx, int rn, const int *src, int sn) {
    int ws = rn / 3, win = rn / 3;
    if (win < 1500) return 1.0;
    int best = win + 1;
    for (int off = -2000; off < 6000; off++) {
        int e = 0, cnt = 0;
        for (int i = 0; i < win; i++) {
            int si = ws + i + off;
            if (si < 0 || si >= sn) continue;
            cnt++;
            if (rx[ws + i] != src[si]) if (++e >= best) break;
        }
        if (cnt > 1000 && e < best) best = e;
    }
    return (double)best / win;
}

int main(int argc, char **argv) {
    int rate = (argc > 1) ? atoi(argv[1]) : 2400;

    v22bis_t cmodem, amodem;
    v22bis_init(&cmodem, rate, 1, V22_GUARD_NONE, 0, call_get, NULL, call_put, NULL, 12000);
    v22bis_init(&amodem, rate, 0, V22_GUARD_NONE, 0, ans_get,  NULL, ans_put,  NULL, 12000);

    int16_t cbuf[40], abuf[40];
    int a_up = -1, c_up = -1;
    for (int t = 0; t < 8000 * 9; t += 40) {
        v22bis_tx(&cmodem, cbuf, 40);
        v22bis_tx(&amodem, abuf, 40);
        v22bis_rx(&amodem, cbuf, 40);
        v22bis_rx(&cmodem, abuf, 40);
        if (a_up < 0 && v22bis_rx_trained(&amodem)) a_up = t;
        if (c_up < 0 && v22bis_rx_trained(&cmodem)) c_up = t;
        if (a_up >= 0 && c_up >= 0 && !ctx_tx.cap) {     /* both up: start capturing */
            ctx_tx.cap = atx_tx.cap = crx.cap = arx.cap = 1;
        }
    }

    int neg = v22bis_current_bit_rate(&amodem);
    if (a_up < 0 || c_up < 0) {
        printf("rate=%d: handshake incomplete (ans_up=%d call_up=%d)  FAIL\n",
               rate, a_up >= 0, c_up >= 0);
        return 1;
    }
    double ber_ca = ber(arx.rx, arx.rn, ctx_tx.src, ctx_tx.sn);   /* call -> ans */
    double ber_ac = ber(crx.rx, crx.rn, atx_tx.src, atx_tx.sn);   /* ans  -> call */
    int ok = (ber_ca < 1e-3) && (ber_ac < 1e-3);
    printf("rate=%d negotiated=%d  call_trained@%dms ans_trained@%dms  "
           "BER call->ans=%.6f  ans->call=%.6f  %s\n",
           rate, neg, c_up / 8, a_up / 8, ber_ca, ber_ac, ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
