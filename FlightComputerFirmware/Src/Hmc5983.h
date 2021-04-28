#ifndef HMC5983_H
#define HMC5983_H

#include <stdint.h>
#include <stdbool.h>

enum HMC5983_MEASUREMENT_MODE
{
  HMC5983_MEASUREMENT_MODE_NORMAL = 0 << 0,
  HMC5983_MEASUREMENT_MODE_POSITIVE_BIAS = 1 << 0,
  HMC5983_MEASUREMENT_MODE_NEGATIVE_BIAS = 2 << 0,
  HMC5983_MEASUREMENT_MODE_TEMPERATURE = 3 << 0,
};

enum HMC5983_DATA_OUTPUT_RATE
{
  HMC5983_DATA_OUTPUT_RATE_0_75 = 0 << 2,
  HMC5983_DATA_OUTPUT_RATE_1_5 = 1 << 2,
  HMC5983_DATA_OUTPUT_RATE_3_0 = 2 << 2,
  HMC5983_DATA_OUTPUT_RATE_7_5 = 3 << 2,
  HMC5983_DATA_OUTPUT_RATE_15_0 = 4 << 2,
  HMC5983_DATA_OUTPUT_RATE_30_0 = 5 << 2,
  HMC5983_DATA_OUTPUT_RATE_75_0 = 6 << 2,
  HMC5983_DATA_OUTPUT_RATE_220_0 = 7 << 2,
};

enum HMC5983_TEMPERATURE_ENABLE
{
  HMC5983_TEMPERATURE_ENABLE_ENABLE = 1 << 7,
  HMC5983_TEMPERATURE_ENABLE_DISABLE = 0 << 7,
};

enum HMC5983_SAMPLES_AVERAGED
{
  HMC5983_SAMPLES_AVERAGED_1 = 0 << 5,
  HMC5983_SAMPLES_AVERAGED_2 = 1 << 5,
  HMC5983_SAMPLES_AVERAGED_4 = 2 << 5,
  HMC5983_SAMPLES_AVERAGED_8 = 3 << 5,
};

enum HMC5983_GAIN
{
  HMC5983_GAIN_0 = 0 << 5,
  HMC5983_GAIN_1 = 1 << 5,
  HMC5983_GAIN_2 = 2 << 5,
  HMC5983_GAIN_3 = 3 << 5,
  HMC5983_GAIN_4 = 4 << 5,
  HMC5983_GAIN_5 = 5 << 5,
  HMC5983_GAIN_6 = 6 << 5,
  HMC5983_GAIN_7 = 7 << 5,
};

enum HMC5983_OPERATING_MODE
{
  HMC5983_OPERATING_MODE_CONTINUOUS = 0 << 0,
  HMC5983_OPERATING_MODE_SINGLE = 1 << 0,
  HMC5983_OPERATING_MODE_IDLE = 3 << 0,
};

#define HMC5983_ADDRESS 0x3D

typedef struct
{
  uint8_t address;
  bool (*write)(const uint8_t, const uint8_t, const uint8_t*, const uint32_t);
  bool (*read)(const uint8_t, const uint8_t, uint8_t* const, const uint32_t);
} Hmc5983Instance_t;

void Hmc5983_Init(Hmc5983Instance_t* const i);
bool Hmc5983_CheckIdentification(Hmc5983Instance_t* const i);
bool Hmc5983_SetConfigA(Hmc5983Instance_t* const i, const uint8_t config_a);
bool Hmc5983_SetConfigB(Hmc5983Instance_t* const i, const uint8_t config_b);
bool Hmc5983_SetMode(Hmc5983Instance_t* const i, const uint8_t mode);
bool Hmc5983_GetMag(Hmc5983Instance_t* const i, int16_t* const x, int16_t* const y, int16_t* const z);
bool Hmc5983_GetTemperature(Hmc5983Instance_t* const i, int16_t* const temperature);

#endif
