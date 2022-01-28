#ifndef MPU6000
#define MPU6000

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  bool (*xaction)(const uint8_t* write, uint8_t* const read, const uint32_t size);
  void (*delay)(const uint32_t us);
} Mpu6000Instance_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} Mpu6000AccelXyz_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} Mpu6000GyroXyz_t;

bool Mpu6000_Init(const Mpu6000Instance_t* i);
bool Mpu6000_ReadAccelTempGyro(const Mpu6000Instance_t* i, Mpu6000AccelXyz_t* const accel, int16_t* const temp, Mpu6000GyroXyz_t* const gyro);


#endif