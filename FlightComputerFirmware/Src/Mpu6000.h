#ifndef MPU6000
#define MPU6000

typedef struct
{
  bool (*xaction)(const uint8_t* write, uint8_t* const read, const uint32_t size);
  void (*delay)(const uint32_t us);
} Mpu6000Instance_t;

bool Mpu6000_Init(const Mpu6000Instance_t* i);


#endif