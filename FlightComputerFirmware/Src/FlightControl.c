#include <stdio.h>
#include <stdbool.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Retarget.h"
#include "Leds.h"
#include "SpektrumRcIn.h"
#include "ServoOut.h"
/*
void FlightControl_Init(void)
{
}

static void FlightControlTask(void* arg)
{
  bool spektrum_status_valid = false;
  SpektrumRcInStatus_t spektrum_status = {0};
  
  while (true)
  {
    spektrum_status_valid = SpectrumRcIn_GetStatus(spektrum_status);
    
    vTaskDelay(SPEKTRUMRCIN_PERIOD);
  }
}

*/