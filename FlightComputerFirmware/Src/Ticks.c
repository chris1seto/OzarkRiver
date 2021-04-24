#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

void Ticks_Reset(TickType_t* const timer)
{
  *timer = xTaskGetTickCount();
}

bool Ticks_IsExpired(const TickType_t ticksStart, const TickType_t timeout)
{
  if ((xTaskGetTickCount() - ticksStart) < timeout)
  {
    return false;
  }

  return true;
}

bool Ticks_OccuredWithin(const TickType_t x, const TickType_t y, const TickType_t timeout)
{
  if ((x - y) <= timeout)
  {
    return true;
  }

  if ((y - x) <= timeout)
  {
    return true;
  }

  return false;
}

TickType_t Ticks_Diff(const TickType_t x, const TickType_t y)
{
  if (x > y)
  {
    return x -y;
  }
  else if (y > x)
  {
    return y - x;
  }
  
  return 0;
}

TickType_t Ticks_Now(void)
{
  return xTaskGetTickCount();
}
