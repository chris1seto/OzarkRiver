#include <stdint.h>
#include <stdbool.h>
#include "Bits.h"

inline bool Bits_IsSet(const uint32_t x, const uint32_t y)
{
  return (x & y) == y;
}

inline void Bits_Set(uint32_t* const x, const uint32_t y)
{
  *x |= y;
}

inline void Bits_Clear(uint32_t* const x, const uint32_t y)
{
  *x &= ~y;
}
