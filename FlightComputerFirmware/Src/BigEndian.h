#ifndef BIGENDIAN_H
#define BIGENDIAN_H

#include <stdint.h>

void BigEndian_Pack16(const uint16_t x, uint8_t* const buffer);
void BigEndian_Pack32(const uint32_t x, uint8_t* const buffer);
void BigEndian_Pack64(const uint64_t x, uint8_t* const buffer);
void BigEndian_Unpack16(const uint8_t* buffer, uint16_t* const x);
void BigEndian_Unpack32(const uint8_t* buffer, uint32_t* const x);
void BigEndian_Unpack64(const uint8_t* buffer, uint64_t* const x);

#endif
