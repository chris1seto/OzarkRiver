#ifndef GEOTOOLS_H
#define GEOTOOLS_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "GeoTypes.h"
#include "GeoTools.h"

enum GEO_COMPUTATION_TYPE
{
  GEO_COMPUTATION_TYPE_HAVERSINE,
  GEO_COMPUTATION_TYPE_VINCENTY,
};

typedef struct
{
  float initial_bearing_deg;
  float final_bearing_deg;
  float distance_ft;
} VincentyInverseResult_t;

typedef struct
{
  float initial_bearing_deg;
  float final_bearing_deg;
  float distance_ft;
  enum GEO_COMPUTATION_TYPE computation_type;
} GeoToolsSegmentData_t;

bool GeoTools_IsWithinPolygon(const Coordinate_t* a, const Coordinate_t* polygon, const uint32_t polygon_size);
float GeoTools_HaversineInitialBearingTo(const Coordinate_t* a, const Coordinate_t* b);
float GeoTools_HaversineFinalBearingTo(const Coordinate_t* a, const Coordinate_t* b);
float GeoTools_HaversineDistanceFt(const Coordinate_t* a, const Coordinate_t* b);
float GeoTools_HaversineCrossTrackErrorFt(const Coordinate_t* a, const Coordinate_t* b, const Coordinate_t* from);
float GeoTools_HeadingError(const float from, const float to);
float GeoTools_AddHeading(const float x, const float y);
bool GeoTools_VincentyInverse(const Coordinate_t* a, const Coordinate_t* b, VincentyInverseResult_t* const result);
bool GeoTools_CalculateSegmentData(const Coordinate_t* a, const Coordinate_t* b, const enum GEO_COMPUTATION_TYPE computation_type, GeoToolsSegmentData_t* const segment_data);
bool GeoTools_VincentyCrossTrackErrorFt(const Coordinate_t* a, const Coordinate_t* b, const Coordinate_t* from, float* const error);
bool GeoTools_HaversineIsCoordinateInFront(const Coordinate_t* from, const float from_heading, const Coordinate_t* target);
bool GeoTools_PrecomputedIsCoordinateInFront(const float bearing_to_target, const float segment_final_bearing);

#endif