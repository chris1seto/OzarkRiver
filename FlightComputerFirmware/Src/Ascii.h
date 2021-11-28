#ifndef ASCII_H
#define ASCII_H

#include <stdint.h>

uint8_t Ascii_Hex2int(const uint8_t x);
float Ascii_atofl(const uint8_t* s, const uint32_t length);
int32_t Ascii_atoil(const uint8_t* buffer, const uint32_t length);
uint8_t Ascii_Int2HexDigit(const uint8_t x);
uint8_t Ascii_IsAscii(const uint8_t x);

#endif
