#include <stm32f3xx_hal.h>

static IWDG_HandleTypeDef watchdogHandle;

void Watchdog_Init(void)
{
  // Init watchdog
  watchdogHandle.Init.Prescaler = IWDG_PRESCALER_16;
  watchdogHandle.Init.Reload = 0x0fff;
  watchdogHandle.Instance = IWDG;
  HAL_IWDG_Init(&watchdogHandle);
}

void Watchdog_Refresh(void)
{
 __HAL_IWDG_RELOAD_COUNTER(&watchdogHandle);
}
