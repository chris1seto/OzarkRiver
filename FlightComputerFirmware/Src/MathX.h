#ifndef MATHX_H
#define MATHX_H

#include <stdint.h>
#include <math.h>

typedef struct
{
  float minimum;
  float maximum;
} IntervalF_t;

uint32_t MathX_BinPiecewise(const float x, const IntervalF_t* intervals, const uint32_t interval_count);
float MathX_ConstrainF(const float x, const float min, const float max);
int32_t MathX_Constrain(const int32_t x, const int32_t min, const int32_t max);
float MathX_MapF(const float x, const float in_min, const float in_max, const float out_min, const float out_max);
float MathX_Magnitude3DF(const float x, const float y, const float z);

#define D_TO_R(x)    ((x * M_PI) / 180.0D)
#define R_TO_D(x)    ((x * 180.0D) / M_PI)

#define IS_NAN(x) (x == NAN)

#endif
