/* Test helper: dump forced-data-mode V.22bis TX samples (s16le @8kHz) to
 * stdout for the offline demodulator check (see test_tx_offline.py).
 * Usage: test_tx_dump [1200|2400] [calling|answering] > out.s16 */
#define _GNU_SOURCE
#include "v22bis.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Reproducible xorshift32 PRBS — mirrored in the python checker. */
static uint32_t prbs = 0x1234u;
static int src_get_bit(void *u) {
    (void)u;
    prbs ^= prbs << 13; prbs ^= prbs >> 17; prbs ^= prbs << 5;
    return (int)(prbs & 1u);
}

int main(int argc, char **argv) {
    int rate = (argc > 1) ? atoi(argv[1]) : 2400;
    int calling = (argc > 2 && strcmp(argv[2], "answering") == 0) ? 0 : 1;

    v22bis_t s;
    v22bis_init(&s, rate, calling, V22_GUARD_NONE, 0,
                src_get_bit, NULL, NULL, NULL, 12000);
    /* Skip the handshake: jump straight to data mode at the chosen rate. */
    s.tx.training = V22_TX_STAGE_NORMAL;
    s.negotiated_bit_rate = rate;

    int16_t buf[8000];
    for (int sec = 0; sec < 4; sec++) {     /* 4 s of signal */
        v22bis_tx(&s, buf, 8000);
        fwrite(buf, sizeof(int16_t), 8000, stdout);
    }
    return 0;
}
