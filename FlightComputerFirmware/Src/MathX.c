#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "MathX.h"

int32_t MathX_Constrain(const int32_t x, const int32_t min, const int32_t max)
{
  if (x < min)
  {
    return min;
  }
  else if (x > max)
  {
    return max;
  }
  
  return x;
}

float MathX_ConstrainF(const float x, const float min, const float max)
{
  if (x < min)
  {
    return min;
  }
  else if (x > max)
  {
    return max;
  }
  
  return x;
}

float MathX_MapF(const float x, const float in_min, const float in_max, const float out_min, const float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float MathX_Magnitude3DF(const float x, const float y, const float z)
{
  return sqrt(x*x + y*y + z*z);
}
