#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Hmc5983.h"
#include "BigEndian.h"

#define HMC_REG_CONFIGURATION_A 0x00
#define HMC_REG_CONFIGURATION_B 0x01
#define HMC_REG_MODE            0x02
#define HMC_REG_XZY_BASE        0x03
#define HMC_REG_STATUS          0x09
#define HMC_REG_IDENTIFICATION_A 0x0a
#define HMC_REG_IDENTIFICATION_B 0x0b
#define HMC_REG_IDENTIFICATION_C 0x0c
#define HMC_REG_TEMPERATURE_BASE 0x31

void Hmc5983_Init(Hmc5983Instance_t* const i)
{
}

bool Hmc5983_CheckIdentification(Hmc5983Instance_t* const i)
{
  uint8_t identification[3];
  uint8_t identification_correct[3] = {'H', '4', '3'};

  if (!i->read(i->address, HMC_REG_IDENTIFICATION_A, (uint8_t*)&identification, 3))
  {
    return false;
  }

  if (memcmp(identification, identification_correct, 3) != 0)
  {
    return false;
  }

  return true;
}

bool Hmc5983_SetConfigA(Hmc5983Instance_t* const i, const uint8_t config_a)
{
  uint8_t readback;

  if (!i->write(i->address, HMC_REG_CONFIGURATION_A, (uint8_t*)&config_a, 1))
  {
    return false;
  }

  if (!i->read(i->address, HMC_REG_CONFIGURATION_A, (uint8_t*)&readback, 1))
  {
    return false;
  }

  if (readback != config_a)
  {
    return false;
  }

  return true;
}

bool Hmc5983_SetConfigB(Hmc5983Instance_t* const i, const uint8_t config_b)
{
  uint8_t readback;

  if (!i->write(i->address, HMC_REG_CONFIGURATION_B, (uint8_t*)&config_b, 1))
  {
    return false;
  }

  if (!i->read(i->address, HMC_REG_CONFIGURATION_B, (uint8_t*)&readback, 1))
  {
    return false;
  }

  if (readback != config_b)
  {
    return false;
  }

  return true;
}

bool Hmc5983_SetMode(Hmc5983Instance_t* const i, const uint8_t mode)
{
  uint8_t readback;

  if (!i->write(i->address, HMC_REG_MODE, (uint8_t*)&mode, 1))
  {
    return false;
  }

  if (!i->read(i->address, HMC_REG_MODE, (uint8_t*)&readback, 1))
  {
    return false;
  }

  if (readback != mode)
  {
    return false;
  }

  return true;
}

bool Hmc5983_GetMag(Hmc5983Instance_t* const i, int16_t* const x, int16_t* const y, int16_t* const z)
{
  uint8_t xzy[sizeof(int16_t) * 3];

  if (!i->read(i->address, HMC_REG_XZY_BASE, (uint8_t*)&xzy, 6))
  {
    return false;
  }

  BigEndian_Unpack16(&xzy[0], (uint16_t*)x);
  BigEndian_Unpack16(&xzy[2], (uint16_t*)z);
  BigEndian_Unpack16(&xzy[4], (uint16_t*)y);

  return true;
}

bool Hmc5983_GetTemperature(Hmc5983Instance_t* const i, int16_t* const temperature)
{
  uint8_t temperature_reg[sizeof(int16_t)];

  if (!i->read(i->address, HMC_REG_TEMPERATURE_BASE, (uint8_t*)&temperature_reg, 6))
  {
    return false;
  }

  BigEndian_Unpack16(&temperature_reg[0], (uint16_t*)temperature);

  return true;
}
