#ifndef NMEA0183_H
#define NMEA0183_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "QueueBuffer.h"

typedef struct
{
  uint8_t hour;
  uint8_t minute;
  float second;
} NmeaTime_t;

typedef struct
{
  uint8_t day;
  uint8_t month;
  uint8_t year;
} NmeaDate_t;

typedef struct
{
  float lat;
  float lon;
} NmeaPosition_t;

typedef struct
{
  NmeaTime_t time;
  NmeaPosition_t position;
  uint8_t fix;
  int32_t sat_count;
  float h_dop;
  float altitude;
  uint8_t altitude_units;
} NmeaGga_t;

typedef struct
{
  NmeaTime_t time;
  uint8_t fix;
  NmeaPosition_t position;
  float speed;
  float track;
  NmeaDate_t date;
} NmeaRmc_t;

typedef struct
{
  NmeaTime_t time;
  uint8_t day;
  uint8_t month;
  uint32_t year;
  uint8_t valid;
} NmeaZda_t;

enum NMEA_MESSAGE_TYPE
{
  NMEA_MESSAGE_TYPE_GGA,
  NMEA_MESSAGE_TYPE_RMC,
  NMEA_MESSAGE_TYPE_GSA,
  NMEA_MESSAGE_TYPE_ZDA
};

typedef struct
{
  enum NMEA_MESSAGE_TYPE message_type;
  union
  {
    NmeaGga_t gga;
    NmeaRmc_t rmc;
    NmeaZda_t zda;
  };
} GenericNmeaMessage_t;

#define MAX_NMEA_SENTENCE_SIZE 256

typedef struct
{
  QueueHandle_t* nmea_message_queue;
  QueueBuffer_t* nmea_queue;
  uint32_t parser_state;
  uint32_t parse_ptr;
  uint8_t packet_buffer[MAX_NMEA_SENTENCE_SIZE];
  uint8_t frame_checksum;
  uint32_t buffer_ptr;
} Nmea0183Instance_t;

void Nmea0183_Init(Nmea0183Instance_t* const inst);
void Nmea0183_Process(Nmea0183Instance_t* const inst);

#endif