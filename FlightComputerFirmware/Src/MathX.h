#ifndef MATHX_H
#define MATHX_H

#include <stdint.h>
#include <math.h>

float MathX_ConstrainF(const float x, const float min, const float max);
float MathX_MapF(const float x, const float in_min, const float in_max, const float out_min, const float out_max);
float MathX_Magnitude3DF(const float x, const float y, const float z);

#define D_TO_R(x)    ((x * M_PI) / 180.0D)
#define R_TO_D(x)    ((x * 180.0D) / M_PI)

#define IS_NAN(x) (x == NAN)

#endif
