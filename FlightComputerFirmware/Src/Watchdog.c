#include <stm32f4xx_hal.h>

static IWDG_HandleTypeDef watchdog_handle;

void Watchdog_Init(void)
{
  // Init watchdog
  watchdog_handle.Instance = IWDG;
  watchdog_handle.Init.Prescaler = IWDG_PRESCALER_4;
  watchdog_handle.Init.Reload = 0x0fff;
  
  HAL_IWDG_Init(&watchdog_handle);
}

void Watchdog_Refresh(void)
{
  __HAL_IWDG_RELOAD_COUNTER(&watchdog_handle);
}
