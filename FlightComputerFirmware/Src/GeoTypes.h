#ifndef GEOTYPES_H
#define GEOTYPES_H

#include <stdint.h>

#define COORDINATE_NAME_SIZE 20
typedef struct
{
	float latitude;
	float longitude;
	float altitude;
} Coordinate_t;

#endif