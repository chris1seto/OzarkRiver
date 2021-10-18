#ifndef CRSF_H
#define CRSF_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#define CRSF_CHANNEL_COUNT 16

typedef struct
{
  TickType_t timestamp;
  float channels[CRSF_CHANNEL_COUNT];
} CrsfChannelData_t;

typedef struct
{
  CrsfChannelData_t channel_data;
} CrsfStatus_t;

void Crsf_Init(void);
bool Crsf_GetStatus(CrsfStatus_t* const status);

#endif