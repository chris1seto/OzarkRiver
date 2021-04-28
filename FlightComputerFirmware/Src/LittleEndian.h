#ifndef LITTLEENDIAN_H
#define LITTLEENDIAN_H

#include <stdint.h>

void LittleEndian_Pack16(const uint16_t x, uint8_t* const buffer);
void LittleEndian_Pack32(const uint32_t x, uint8_t* const buffer);
void LittleEndian_Pack64(const uint64_t x, uint8_t* const buffer);
void LittleEndian_Unpack16(const uint8_t* buffer, uint16_t* const x);
void LittleEndian_Unpack32(const uint8_t* buffer, uint32_t* const x);
void LittleEndian_Unpack64(const uint8_t* buffer, uint64_t* const x);

#endif
