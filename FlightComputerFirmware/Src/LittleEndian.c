#include <stdint.h>
#include "LittleEndian.h"

void LittleEndian_Pack16(const uint16_t x, uint8_t* const buffer)
{
	uint8_t* pcontent = (uint8_t*)&x;

	buffer[0] = pcontent[0];
  buffer[1] = pcontent[1];
}

void LittleEndian_Pack32(const uint32_t x, uint8_t* const buffer)
{
	uint8_t* pcontent = (uint8_t*)&x;

	buffer[0] = pcontent[0];
  buffer[1] = pcontent[1];
  buffer[2] = pcontent[2];
  buffer[3] = pcontent[3];
}

void LittleEndian_Pack64(const uint64_t x, uint8_t* const buffer)
{
	uint8_t* pcontent = (uint8_t*)&x;

	buffer[0] = pcontent[0];
  buffer[1] = pcontent[1];
  buffer[2] = pcontent[2];
  buffer[3] = pcontent[3];
  buffer[4] = pcontent[4];
  buffer[5] = pcontent[5];
  buffer[6] = pcontent[6];
  buffer[7] = pcontent[7];
}

void LittleEndian_Unpack16(const uint8_t* buffer, uint16_t* const x)
{
	uint8_t* pcontent = (uint8_t*)x;

	pcontent[0] = buffer[0];
	pcontent[1] = buffer[1];
}

void LittleEndian_Unpack32(const uint8_t* buffer, uint32_t* const x)
{
	uint8_t* pcontent = (uint8_t*)x;

	pcontent[0] = buffer[0];
	pcontent[1] = buffer[1];
	pcontent[2] = buffer[2];
	pcontent[3] = buffer[3];
}

void LittleEndian_Unpack64(const uint8_t* buffer, uint64_t* const x)
{
	uint8_t* pcontent = (uint8_t*)x;

	pcontent[0] = buffer[0];
	pcontent[1] = buffer[1];
	pcontent[2] = buffer[2];
	pcontent[3] = buffer[3];
	pcontent[4] = buffer[4];
	pcontent[5] = buffer[5];
	pcontent[6] = buffer[6];
	pcontent[7] = buffer[7];
}
