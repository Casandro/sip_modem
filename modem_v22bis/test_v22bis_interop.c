/* Interop verification against the spandsp reference V.22bis modem.
 *
 * My modem mirrors spandsp's public API names, so they collide at link time;
 * my symbols are aliased to mine_* here (and my .c files are compiled with the
 * matching -D defines — see the Makefile `test` target). My modem is run in
 * spandsp_compat mode so the data scrambler matches spandsp's (spandsp uses the
 * calling polynomial in both directions, unlike the ITU spec).
 *
 * Runs my modem against spandsp's, full-duplex, in both role pairings and at
 * both rates, and checks both modems train and exchange a PRBS with low BER. */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Alias my API away from spandsp's identical names. */
#define v22bis_init             mine_v22bis_init
#define v22bis_tx               mine_v22bis_tx
#define v22bis_rx               mine_v22bis_rx
#define v22bis_rx_restart       mine_v22bis_rx_restart
#include "v22bis.h"
#undef v22bis_init
#undef v22bis_tx
#undef v22bis_rx
#undef v22bis_rx_restart

#include <spandsp.h>

#define MAXBITS 800000
typedef struct { uint32_t r; int *src; int n, cap; } prbs_t;
static int prbs_bit(prbs_t *p) {
    p->r ^= p->r << 13; p->r ^= p->r >> 17; p->r ^= p->r << 5;
    int b = (int)(p->r & 1u);
    if (p->cap && p->n < MAXBITS) p->src[p->n++] = b;
    return b;
}
typedef struct { int *rx; int n, cap; } sink_t;
static void sink_put(sink_t *s, int bit) {
    if (bit < 0) return;                 /* spandsp status report, not data */
    if (s->cap && s->n < MAXBITS) s->rx[s->n++] = bit & 1;
}

/* Two PRBS sources and two sinks, shared by both role pairings (reset each run). */
static prbs_t A_src, B_src;       /* A = my modem's data, B = spandsp's data */
static sink_t A_sink, B_sink;     /* A_sink = spandsp decoding A; B_sink = mine decoding B */
static int mine_get(void *u){ (void)u; return prbs_bit(&A_src); }
static void mine_put(void *u, int b){ (void)u; sink_put(&B_sink, b); }
static int sp_get(void *u){ (void)u; return prbs_bit(&B_src); }
static void sp_put(void *u, int b){ (void)u; sink_put(&A_sink, b); }

static double ber(const int *rx, int rn, const int *src, int sn) {
    int ws = rn / 3, win = rn / 3;
    if (win < 1500) return 1.0;
    int best = win + 1;
    for (int off = -3000; off < 8000; off++) {
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

/* mine_calling: 1 if my modem is the calling party (spandsp answers), else 0. */
static int run(int rate, int mine_calling) {
    static int a_s[MAXBITS], b_s[MAXBITS], a_r[MAXBITS], b_r[MAXBITS];
    A_src = (prbs_t){0x1234u, a_s, 0, 0};
    B_src = (prbs_t){0x9e3779b9u, b_s, 0, 0};
    A_sink = (sink_t){a_r, 0, 0};
    B_sink = (sink_t){b_r, 0, 0};

    v22bis_t mine;
    mine_v22bis_init(&mine, rate, mine_calling, V22_GUARD_NONE, 1 /*spandsp_compat*/,
                     mine_get, NULL, mine_put, NULL, 12000);
    v22bis_state_t *sp = v22bis_init(NULL, rate, V22BIS_GUARD_TONE_NONE,
                                     mine_calling ? 0 : 1, sp_get, NULL, sp_put, NULL);
    if (!sp) { printf("spandsp init failed\n"); return 1; }

    int16_t mb[40], sb[40];
    int trained = -1;
    for (int t = 0; t < 8000 * 12; t += 40) {
        mine_v22bis_tx(&mine, mb, 40);
        v22bis_tx(sp, sb, 40);
        v22bis_rx(sp, mb, 40);
        mine_v22bis_rx(&mine, sb, 40);
        int up = v22bis_rx_trained(&mine) && (v22bis_get_current_bit_rate(sp) > 0);
        if (trained < 0 && up) {
            trained = t;
            A_src.cap = B_src.cap = A_sink.cap = B_sink.cap = 1;
        }
    }
    int mine_rate = v22bis_current_bit_rate(&mine);
    int sp_rate = v22bis_get_current_bit_rate(sp);
    double ber_m2s = ber(a_r, A_sink.n, a_s, A_src.n);  /* mine -> spandsp */
    double ber_s2m = ber(b_r, B_sink.n, b_s, B_src.n);  /* spandsp -> mine */
    int ok = trained >= 0 && ber_m2s < 5e-3 && ber_s2m < 5e-3;
    printf("  %-13s rate=%d: mine_rate=%d sp_rate=%d  BER mine->sp=%.5f sp->mine=%.5f  %s\n",
           mine_calling ? "mine=calling" : "mine=answer", rate, mine_rate, sp_rate,
           ber_m2s, ber_s2m, ok ? "PASS" : "FAIL");
    v22bis_free(sp);
    return ok ? 0 : 1;
}

int main(void) {
    int rc = 0;
    printf("V.22bis interop vs spandsp:\n");
    for (int r = 0; r < 2; r++) {
        int rate = r ? 2400 : 1200;
        rc |= run(rate, 1);   /* my modem calling, spandsp answering */
        rc |= run(rate, 0);   /* my modem answering, spandsp calling */
    }
    return rc;
}
