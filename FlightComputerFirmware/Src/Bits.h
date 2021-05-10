#ifndef BITS_H
#define BITS_H

#include <stdint.h>
#include <stdbool.h>

extern bool Bits_IsSet(const uint32_t x, const uint32_t y);
extern void Bits_Set(uint32_t* const x, const uint32_t y);
extern void Bits_Clear(uint32_t* const x, const uint32_t y);


#endif
