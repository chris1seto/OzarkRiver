#ifndef HMC5983_H
#define HMC5983_H

#include <stdint.h>
#include <stdbool.h>

enum 

typedef struct
{
  uint8_t address;
  bool (*write)(const uint8_t dev_addr, const uint8_t* data, const uint32_t size);
  bool (*read)(const uint8_t dev_addr, uint8_t* const data, const uint32_t size);
} Hmc5983Instance_t;

#endif
