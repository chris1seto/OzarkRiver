#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "MathX.h"
#include "GeoTypes.h"
#include "GeoTools.h"

// WS84 feet
static const float e_a = 20925646.33f;
static const float e_f = 1.0f / 298.257223563f;
static const float e_b = 20855486.59529199f;
#define VINCENTY_MAX_ITERATION_COUNT 1000

static float Angle2D(const float y1, const float x1, const float y2, const float x2);
static float sq(const float x);
static float FixHeading(const float x);

static float sq(const float x)
{
  return x * x;
}

static float FixHeading(const float x)
{
  // Fix heading from the ins in [-180, 180] to (0, 360]
  if (x < 0)
  {
    return x + 360.0f;
  }

  return x;
}

bool GeoTools_CalculateSegmentData(const Coordinate_t* a, const Coordinate_t* b, const enum GEO_COMPUTATION_TYPE computation_type, GeoToolsSegmentData_t* const segment_data)
{
  VincentyInverseResult_t vincenty_result = {0};

  // First attempt vincenty
  if (computation_type == GEO_COMPUTATION_TYPE_VINCENTY)
  {
    if (GeoTools_VincentyInverse(a, b, &vincenty_result))
    {
      segment_data->initial_bearing_deg = vincenty_result.initial_bearing_deg;
      segment_data->final_bearing_deg = vincenty_result.final_bearing_deg;
      segment_data->distance_ft = vincenty_result.distance_ft;
      segment_data->computation_type = GEO_COMPUTATION_TYPE_VINCENTY;

      if (!IS_NAN(segment_data->initial_bearing_deg)
        && !IS_NAN(segment_data->final_bearing_deg)
        && !IS_NAN(segment_data->distance_ft))
      {
        return true;
      }
    }
  }

  // Drop back to haversine
  segment_data->initial_bearing_deg = GeoTools_HaversineInitialBearingTo(a, b);
  segment_data->final_bearing_deg = GeoTools_HaversineFinalBearingTo(a, b);
  segment_data->distance_ft = GeoTools_HaversineDistanceFt(a, b);
  segment_data->computation_type = GEO_COMPUTATION_TYPE_HAVERSINE;

  if (IS_NAN(segment_data->initial_bearing_deg)
    || IS_NAN(segment_data->final_bearing_deg)
    || IS_NAN(segment_data->distance_ft))
  {
    return false;
  }

  return true;
}

float GeoTools_AddHeading(const float x, const float y)
{
  float result;

  result = fmodf((x + y), 360.0f);

  if (result == 0)
  {
    result = 360.0f;
  }

  if (result < 0)
  {
    result += 360.0f;
  }

  return result;
}

// Calculate the error between two headings
// + means the to heading is to the right of the from
// - means the to heading is to the left of the from
float GeoTools_HeadingError(const float from, const float to)
{
  float error;
  float abs_error;

  error = (to - from);
  abs_error = fabs(error);

  if (abs_error < 180.0f)
  {
    return error;
  }

  if (to > from)
  {
    return abs_error - 360.0f;
  }

  return 360.0f - abs_error;
}

bool GeoTools_IsWithinPolygon(const Coordinate_t* a, const Coordinate_t* polygon, const uint32_t polygon_size)
{
  float point_a[2];
  float point_b[2];
  uint32_t i;
  float angle = 0;

  for (i = 0; i < polygon_size; i++)
  {
    point_a[0] = polygon[i].latitude - a->latitude;
    point_a[1] = polygon[i].longitude - a->longitude;

    point_b[0] = polygon[(i + 1) % polygon_size].latitude - a->latitude;
    point_b[1] = polygon[(i + 1) % polygon_size].longitude - a->longitude;

    angle += Angle2D(point_a[0], point_a[1],
      point_b[0], point_b[1]);
  }

  if (fabs(angle) < M_PI)
  {
    return false;
  }

  return true;
}

static float Angle2D(const float y1, const float x1, const float y2, const float x2)
{
  float theta1;
  float theta2;
  float delta_theta;

  theta1 = atan2(y1, x1);
  theta2 = atan2(y2, x2);
  delta_theta = theta2 - theta1;

  while (delta_theta > M_PI)
  {
    delta_theta -= 2.0 * M_PI;
  }

  while (delta_theta < -M_PI)
  {
    delta_theta += 2.0 * M_PI;
  }

  return delta_theta;
}

float GeoTools_HaversineInitialBearingTo(const Coordinate_t* a, const Coordinate_t* b)
{
  float a_lat_r;
  float a_lon_r;
  float b_lat_r;
  float b_lon_r;
  float delta_lon;
  float x;
  float y;
  float result;

  // Convert everything to radians first
  a_lat_r = D_TO_R(a->latitude);
  a_lon_r = D_TO_R(a->longitude);

  b_lat_r = D_TO_R(b->latitude);
  b_lon_r = D_TO_R(b->longitude);

  // Get delta lon
  delta_lon = (b_lon_r - a_lon_r);

  y = sin(delta_lon) * cos(b_lat_r);
  x = (cos(a_lat_r) * sin(b_lat_r)) - (sin(a_lat_r) * cos(b_lat_r) * cos(delta_lon));

  result = fmod((R_TO_D(atan2(y, x)) + 360.0), 360.0);

  if (result == 0)
  {
    result = 360.0f;
  }

  return result;
}

float GeoTools_HaversineFinalBearingTo(const Coordinate_t* a, const Coordinate_t* b)
{
  return fmod((GeoTools_HaversineInitialBearingTo(b, a) + 180.0f), 360.0f);
}

float GeoTools_HaversineDistanceFt(const Coordinate_t* a, const Coordinate_t* b)
{
  float a_lat_r;
  float a_lon_r;
  float b_lat_r;
  float b_lon_r;
  float delta_lat;
  float delta_lon;
  float ea;
  float ec;

  // Convert everything to radians first
  a_lat_r = D_TO_R(a->latitude);
  a_lon_r = D_TO_R(a->longitude);

  b_lat_r = D_TO_R(b->latitude);
  b_lon_r = D_TO_R(b->longitude);

  delta_lat = b_lat_r - a_lat_r;
  delta_lon = b_lon_r - a_lon_r;

  ea = (sin(delta_lat / 2.0) * sin(delta_lat / 2.0)) + (cos(a_lat_r) * cos(b_lat_r) * sin(delta_lon / 2.0) * sin(delta_lon / 2.0));

  ec = 2.0 * atan2(sqrt(ea), sqrt(1.0 - ea));

  return e_a * ec;
}

// Calculate cross track error
// - is left of a -> b segment
// + is right of a -> b segment
float GeoTools_HaversineCrossTrackErrorFt(const Coordinate_t* a, const Coordinate_t* b, const Coordinate_t* from)
{
  float distance;
  float a_to_from_bearing;
  float a_to_b_bearing;

  a_to_from_bearing = D_TO_R(GeoTools_HaversineInitialBearingTo(a, from));
  a_to_b_bearing = D_TO_R(GeoTools_HaversineInitialBearingTo(a, b));
  distance = GeoTools_HaversineDistanceFt(a, from) / e_a;

  return asin(sin(distance) * sin(a_to_from_bearing - a_to_b_bearing)) * e_a;
}

// Calculate cross track error
// - is left of a -> b segment
// + is right of a -> b segment
bool GeoTools_VincentyCrossTrackErrorFt(const Coordinate_t* a, const Coordinate_t* b, const Coordinate_t* from, float* const error)
{
  VincentyInverseResult_t vincenty_result;
  float distance;
  float a_to_from_bearing;
  float a_to_b_bearing;

  // a -> from segment
  if (!GeoTools_VincentyInverse(a, from, &vincenty_result))
  {
    return false;
  }

  a_to_from_bearing = D_TO_R(vincenty_result.initial_bearing_deg);
  distance = vincenty_result.distance_ft / e_a;

  // a -> b segment
  if (!GeoTools_VincentyInverse(a, b, &vincenty_result))
  {
    return false;
  }

  a_to_b_bearing = D_TO_R(vincenty_result.initial_bearing_deg);

  *error = asin(sin(distance) * sin(a_to_from_bearing - a_to_b_bearing)) * e_a;

  return true;
}

bool GeoTools_HaversineIsCoordinateInFront(const Coordinate_t* from, const float from_heading, const Coordinate_t* target)
{
  float bearing_to;
  float heading_error;

  bearing_to = GeoTools_HaversineInitialBearingTo(from, target);

  heading_error = GeoTools_HeadingError(bearing_to, from_heading);

  if (heading_error <= -90.0 || heading_error >= 90.0)
  {
    return true;
  }

  return false;
}

bool GeoTools_PrecomputedIsCoordinateInFront(const float bearing_to_target, const float segment_final_bearing)
{
  float heading_error;

  heading_error = GeoTools_HeadingError(bearing_to_target, segment_final_bearing);

  if (heading_error <= -90.0 || heading_error >= 90.0)
  {
    return true;
  }

  return false;
}

void GeoTools_CreateCoordinateFromVector(const Coordinate_t* a, const float distance_ft, Coordinate_t* const new)
{
}

/*
  Source: https://metacpan.org/pod/GIS::Distance::Vincenty

  Algo:
    a, b = major & minor semiaxes of the ellipsoid
    f = flattening (a-b)/a
    L = lon2 - lon1
    u1 = atan((1-f) * tan(lat1))
    u2 = atan((1-f) * tan(lat2))
    sin_u1 = sin(u1)
    cos_u1 = cos(u1)
    sin_u2 = sin(u2)
    cos_u2 = cos(u2)
    lambda = L
    lambda_pi = 2PI
    while abs(lambda-lambda_pi) > 1e-12
        sin_lambda = sin(lambda)
        cos_lambda = cos(lambda)
        sin_sigma = sqrt((cos_u2 * sin_lambda) * (cos_u2*sin_lambda) +
            (cos_u1*sin_u2-sin_u1*cos_u2*cos_lambda) * (cos_u1*sin_u2-sin_u1*cos_u2*cos_lambda))
        cos_sigma = sin_u1*sin_u2 + cos_u1*cos_u2*cos_lambda
        sigma = atan2(sin_sigma, cos_sigma)
        alpha = asin(cos_u1 * cos_u2 * sin_lambda / sin_sigma)
        cos_sq_alpha = cos(alpha) * cos(alpha)
        cos2sigma_m = cos_sigma - 2*sin_u1*sin_u2/cos_sq_alpha
        cc = f/16*cos_sq_alpha*(4+f*(4-3*cos_sq_alpha))
        lambda_pi = lambda
        lambda = L + (1-cc) * f * sin(alpha) *
            (sigma + cc*sin_sigma*(cos2sigma_m+cc*cos_sigma*(-1+2*cos2sigma_m*cos2sigma_m)))
    }
    usq = cos_sq_alpha*(a*a-b*b)/(b*b);
    aa = 1 + usq/16384*(4096+usq*(-768+usq*(320-175*usq)))
    bb = usq/1024 * (256+usq*(-128+usq*(74-47*usq)))
    delta_sigma = bb*sin_sigma*(cos2sigma_m+bb/4*(cos_sigma*(-1+2*cos2sigma_m*cos2sigma_m)-
      bb/6*cos2sigma_m*(-3+4*sin_sigma*sin_sigma)*(-3+4*cos2sigma_m*cos2sigma_m)))
    c = b*aa*(sigma-delta_sigma)
*/
bool GeoTools_VincentyInverse(const Coordinate_t* a, const Coordinate_t* b, VincentyInverseResult_t* const result)
{
  float theta_1;
  float theta_2;
  float sin_sigma = 0;
  float cos_sigma = 0;
  float sigma = 0;
  float delta_sigma;

  float lambda;
  float sin_lambda = 0;
  float cos_lambda = 0;
  uint32_t iterations;
  float l_1;
  float l_2;

  float u_1;
  float u_2;
  float sin_u_1;
  float cos_u_1;
  float sin_u_2;
  float cos_u_2;

  float cos_sq_alpha = 0;
  float cos2sigma_m = 0;
  float u_sq;

  float big_l;
  float big_c;
  float big_a;
  float big_b;

  float alpha;

  float s;
  float alpha_1;
  float alpha_2;

  float lambda_pi;

  if (a->latitude == b->latitude
    && a->longitude == b->longitude)
  {
    result->initial_bearing_deg = 360.0;
    result->final_bearing_deg = 360.0;
    result->distance_ft = 0;
    return true;
  }

  // Initial conditions
  theta_1 = D_TO_R(a->latitude);
  theta_2 = D_TO_R(b->latitude);
  l_1 = D_TO_R(a->longitude);
  l_2 = D_TO_R(b->longitude);

  // OK
  big_l = l_2 - l_1;

  // OK
  u_1 = atan((1.0f - e_f) * tan(theta_1));
  u_2 = atan((1.0f - e_f) * tan(theta_2));

  // OK
  sin_u_1 = sin(u_1);
  cos_u_1 = cos(u_1);
  sin_u_2 = sin(u_2);
  cos_u_2 = cos(u_2);

  lambda = big_l;
  lambda_pi = 2.0f * M_PI;
  iterations = 0;

  // OK
  while (fabs(lambda - lambda_pi) > 1e-12)
  {
    if (iterations++ > VINCENTY_MAX_ITERATION_COUNT)
    {
      return false;
    }

    // OK
    sin_lambda = sin(lambda);
    cos_lambda = cos(lambda);

    // OK
    sin_sigma = sqrt(sq(cos_u_2 * sin_lambda) + sq(cos_u_1 * sin_u_2 - sin_u_1 * cos_u_2 * cos_lambda));

    // OK
    cos_sigma = sin_u_1 * sin_u_2 + cos_u_1 * cos_u_2 * cos_lambda;

    // OK
    sigma = atan2(sin_sigma, cos_sigma);

    // OK
    alpha = asin(cos_u_1 * cos_u_2 * sin_lambda / sin_sigma);

    // OK
    cos_sq_alpha = sq(cos(alpha));

    // OK
    cos2sigma_m = cos_sigma - 2.0f * sin_u_1 * sin_u_2 / cos_sq_alpha;

    // OK
    big_c = (e_f / 16.0f) * cos_sq_alpha * (4.0f + e_f * (4.0f - 3.0f * cos_sq_alpha));

    // OK
    lambda_pi = lambda;

    // OK
    lambda = big_l + (1.0f - big_c) * e_f * sin(alpha) * (sigma + big_c * sin_sigma * (cos2sigma_m + big_c * cos_sigma * (-1.0f + 2.0f * sq(cos2sigma_m))));
  }

  // OK
  u_sq = cos_sq_alpha * ((sq(e_a) - sq(e_b)) / sq(e_b));

  // OK
  big_a = 1.0f + (u_sq / 16384.0f) * (4096.0f + u_sq * (-768.0f + u_sq * (320.0f - 175.0f * u_sq)));

  // OK
  big_b = (u_sq / 1024.0f) * (256.0f + (u_sq * (-128.0f + u_sq * (74.0f - 47.0f * u_sq))));

  // OK
  delta_sigma = big_b * sin_sigma * (cos2sigma_m + (big_b / 4.0f) * (cos_sigma * (-1.0f + 2.0f * sq(cos2sigma_m)) - big_b / 6.0f * cos2sigma_m * (-3.0f + 4.0f * sq(sin_sigma)) * (-3.0f + 4.0f * sq(cos2sigma_m))));

  // OK
  s = e_b * big_a * (sigma - delta_sigma);

  // OK
  alpha_1 = atan2(cos_u_2 * sin_lambda, cos_u_1 * sin_u_2 - sin_u_1 * cos_u_2 * cos_lambda);
  alpha_2 = atan2(cos_u_1 * sin_lambda, -sin_u_1 * cos_u_2 + cos_u_1 * sin_u_2 * cos_lambda);

  result->initial_bearing_deg = FixHeading(R_TO_D(alpha_1));
  result->final_bearing_deg = FixHeading(R_TO_D(alpha_2));
  result->distance_ft = s;

  return true;
}
