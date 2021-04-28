#include <stdint.h>
#include "BigEndian.h"

void BigEndian_Pack16(const uint16_t x, uint8_t* const buffer)
{
	uint8_t* pcontent = (uint8_t*)&x;

	buffer[1] = pcontent[0];
  buffer[0] = pcontent[1];
}

void BigEndian_Pack32(const uint32_t x, uint8_t* const buffer)
{
	uint8_t* pcontent = (uint8_t*)&x;

	buffer[3] = pcontent[0];
  buffer[2] = pcontent[1];
  buffer[1] = pcontent[2];
  buffer[0] = pcontent[3];
}

void BigEndian_Pack64(const uint64_t x, uint8_t* const buffer)
{
	uint8_t* pcontent = (uint8_t*)&x;

	buffer[7] = pcontent[0];
  buffer[6] = pcontent[1];
  buffer[5] = pcontent[2];
  buffer[4] = pcontent[3];
  buffer[3] = pcontent[4];
  buffer[2] = pcontent[5];
  buffer[1] = pcontent[6];
  buffer[0] = pcontent[7];
}

void BigEndian_Unpack16(const uint8_t* buffer, uint16_t* const x)
{
	uint8_t* pcontent = (uint8_t*)x;

	pcontent[1] = buffer[0];
	pcontent[0] = buffer[1];
}

void BigEndian_Unpack32(const uint8_t* buffer, uint32_t* const x)
{
	uint8_t* pcontent = (uint8_t*)x;

	pcontent[3] = buffer[0];
	pcontent[2] = buffer[1];
	pcontent[1] = buffer[2];
	pcontent[0] = buffer[3];
}

void BigEndian_Unpack64(const uint8_t* buffer, uint64_t* const x)
{
	uint8_t* pcontent = (uint8_t*)x;

	pcontent[7] = buffer[0];
	pcontent[6] = buffer[1];
	pcontent[5] = buffer[2];
	pcontent[4] = buffer[3];
	pcontent[3] = buffer[4];
	pcontent[2] = buffer[5];
	pcontent[1] = buffer[6];
	pcontent[0] = buffer[7];
}
