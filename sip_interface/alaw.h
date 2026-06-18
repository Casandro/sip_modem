#ifndef ALAW_H
#define ALAW_H
#include <stdint.h>

void    alaw_init(void);
uint8_t alaw_encode(int16_t pcm);
int16_t alaw_decode(uint8_t a);

#endif
