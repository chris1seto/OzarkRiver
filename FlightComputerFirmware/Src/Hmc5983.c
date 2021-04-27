#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "Hmc5983.h"

#define HMC_REG_CONFIGURATION_A 0x00
#define HMC_REG_CONFIGURATION_B 0x01
#define HMC_REG_MODE            0x02
#define HMC_REG_XZY_BASE        0x03
#define HMC_REG_STATUS          0x09
#define HMC_REG_IDENTIFICATION_A 0x0a
#define HMC_REG_IDENTIFICATION_B 0x0b


void Hmc5983_Init(Hmc5983Instance_t* const i)
{
}

bool Hmc5983_GetMag(Hmc5983Instance_t* const i, int16_t* x, int16_t* y, int16_t* z)
{
  
  return true;
}

bool Hmc5983_GetTemperature(Hmc5983Instance_t* const i, int16_t temperature)
{
  return true;
}