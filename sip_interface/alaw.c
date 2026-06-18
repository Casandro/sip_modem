#include "alaw.h"
#include <stdint.h>

static int16_t dtable[256];

static int16_t decode_one(uint8_t a) {
    a ^= 0xD5;
    int sign = a >> 7;
    int exp  = (a >> 4) & 7;
    int mant = a & 0xF;
    int val;
    if (exp == 0)
        val = (mant << 4) | 8;
    else
        val = ((mant | 0x10) << (exp + 3)) | (1 << (exp + 2));
    /* After XOR 0xD5, bit7=0 means positive (0xD5 has bit7=1, so it flips) */
    return (int16_t)(sign ? -val : val);
}

void alaw_init(void) {
    for (int i = 0; i < 256; i++)
        dtable[i] = decode_one((uint8_t)i);
}

int16_t alaw_decode(uint8_t a) { return dtable[a]; }

uint8_t alaw_encode(int16_t pcm_val) {
    static const int seg_end[8] = {
        0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF, 0x3FFF, 0x7FFF
    };
    int mask, seg;
    uint8_t aval;

    if (pcm_val >= 0) {
        mask = 0xD5;
    } else {
        mask = 0x55;
        pcm_val = (int16_t)(~pcm_val);
    }

    int v = (int)(uint16_t)pcm_val;
    for (seg = 0; seg < 8 && v > seg_end[seg]; seg++);

    if (seg == 0)
        aval = (uint8_t)(v >> 4);
    else
        aval = (uint8_t)((seg << 4) | ((v >> (seg + 3)) & 0xF));

    return aval ^ (uint8_t)mask;
}
