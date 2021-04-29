#ifndef IMUAHRS_H
#define IMUAHRS_H

#include <stdio.h>
#include <stdbool.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
  TickType_t timestamp;
  uint32_t flags;
  float pitch;
  float roll;
  float yaw;
  float pitch_rate;
  float roll_rate;
  float yaw_rate;
} ImuAhrsStatus_t;

void ImuAhrs_Init();
bool ImuAhrs_GetStatus(ImuAhrsStatus_t* const status);

#endif
