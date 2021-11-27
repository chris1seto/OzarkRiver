/*
  Resources
    * http://www.gpsinformation.org/dale/nmea.htm
*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "QueueBuffer.h"
#include "Ascii.h"
#include "TokenIterate.h"
#include "Nmea0183.h"
#include "Ticks.h"

// Sentence processors
static void ParseGsa(Nmea0183Instance_t* const inst, const uint32_t size);
static void ParseGga(Nmea0183Instance_t* const inst, const uint32_t size);
static void ParseRmc(Nmea0183Instance_t* const inst, const uint32_t size);
static void ParseZda(Nmea0183Instance_t* const inst, const uint32_t size);
static void ProcessSentence(Nmea0183Instance_t* const inst, const uint32_t size);

// Packet extractors
static uint8_t ExtractTime(TokenIterate_t* t, NmeaTime_t* time);
static uint8_t ExtractDate(TokenIterate_t* t, NmeaDate_t* date);
static uint8_t ExtractPosition(TokenIterate_t* t, NmeaPosition_t* pos);
static uint8_t ExtractChar(TokenIterate_t* t, uint8_t* c);
static uint8_t ExtractFloat(TokenIterate_t* t, float* x);
static uint8_t ExtractInt(TokenIterate_t* t, int32_t* x);

typedef struct
{
  enum NMEA_MESSAGE_TYPE type;
  char header[5];
  void(*processor)(Nmea0183Instance_t* const, const uint32_t);
} NmeaProcessor_t;

#define PARSE_STATE_HEADER     0
#define PARSE_STATE_PAYLOAD    1
#define PARSE_STATE_CHECKSUM0  2
#define PARSE_STATE_CHECKSUM1  3

// NMEA processors
#define NMEA_PROCESSOR_COUNT  4
static const NmeaProcessor_t processors[NMEA_PROCESSOR_COUNT] =
{
  {NMEA_MESSAGE_TYPE_GSA, "G*GSA", ParseGsa},
  {NMEA_MESSAGE_TYPE_GGA, "G*GGA", ParseGga},
  {NMEA_MESSAGE_TYPE_RMC, "G*RMC", ParseRmc},
  {NMEA_MESSAGE_TYPE_ZDA, "G*ZDA", ParseZda},
  
};

void Nmea0183_Init(Nmea0183Instance_t* const inst)
{
  inst->parser_state = PARSE_STATE_HEADER;
  inst->parse_ptr = 0;
  inst->frame_checksum = 0;
  inst->buffer_ptr = 0;
}

// Verify a packet checksum
static uint8_t VerifyChecksum(const uint8_t x, const uint8_t* buffer, const uint32_t size)
{
  uint8_t checksum = 0;
  uint32_t i = 0;

  for (i = 0; i < size; i++)
  {
    checksum ^= buffer[i];
  }

  return (x == checksum);
}

void Nmea0183_Process(Nmea0183Instance_t* const inst)
{
  uint8_t working_byte;

  // While there is data in the queue
  while (inst->parse_ptr < QueueBuffer_Count(inst->nmea_queue))
  {
    // Get the current ptr
    QueueBuffer_Peek(inst->nmea_queue, inst->parse_ptr++, &working_byte);

    // State machine parser
    switch(inst->parser_state)
    {
      // Look for and consume header byte $
      case PARSE_STATE_HEADER:
        if (working_byte == '$')
        {
          inst->parser_state = PARSE_STATE_PAYLOAD;
        }

        // Dequeue whatever we get at this point, header or not
        QueueBuffer_Dequeue(inst->nmea_queue, 1);
        inst->parse_ptr = 0;
        inst->buffer_ptr = 0;
        break;

      // Wait until we get past the payload
      case PARSE_STATE_PAYLOAD:
        if (working_byte == '*')
        {
          inst->parser_state = PARSE_STATE_CHECKSUM0;
        }
        else
        {
          inst->packet_buffer[inst->buffer_ptr++] = working_byte;
        }
        break;

      // First checksum byte
      case PARSE_STATE_CHECKSUM0:

        // Just collect this byte to start building the checksum
        inst->frame_checksum = (Ascii_Hex2int(working_byte) << 4);

        // Move on
        inst->parser_state = PARSE_STATE_CHECKSUM1;
        break;

      // Second checksum byte
      case PARSE_STATE_CHECKSUM1:
        // Continue building the checksum
        inst->frame_checksum |= Ascii_Hex2int(working_byte);

        // Attempt to verify the checksum
        if (VerifyChecksum(inst->frame_checksum, inst->packet_buffer, inst->buffer_ptr))
        {
          // Dequeue the entire packet if success
          QueueBuffer_Dequeue(inst->nmea_queue, inst->parse_ptr);
          inst->parse_ptr = 0;

          // Process sentence
          ProcessSentence(inst, inst->buffer_ptr);
        }

        // Reset
        inst->parser_state = PARSE_STATE_HEADER;
        break;
    }
  }
}

static uint8_t MatchUpTo(const uint8_t* x, const uint8_t* y, const uint32_t len)
{
  uint32_t i;

  for (i = 0; i < len; i++)
  {
    if (x[i] != '*' && x[i] != y[i])
    {
      return 0;
    }
  }

  return 1;
}

static void ProcessSentence(Nmea0183Instance_t* const inst, const uint32_t size)
{
  uint32_t i;

  // Try to match to a known processor
  for (i = 0; i < NMEA_PROCESSOR_COUNT; i++)
  {
    // Check if we have a match
    if (MatchUpTo((uint8_t*)processors[i].header, inst->packet_buffer, 5))
    {
      // Call the processor
      processors[i].processor(inst, size);
      return;
    }
  }
}

// Extract time
static uint8_t ExtractTime(TokenIterate_t* t, NmeaTime_t* time)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIterator_Forward(t, &token, &token_size);

  // Check size
  if (token_size < 6)
  {
    return 0;
  }

  // Hour
  time->hour = Ascii_atoil(token + 0, 2);

  // Minute
  time->minute = Ascii_atoil(token + 2, 2);

  // Second (might be a float)
  time->second = Ascii_atofl(token + 4, token_size - 4);

  return 1;
}

// Extract date
static uint8_t ExtractDate(TokenIterate_t* t, NmeaDate_t* date)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIterator_Forward(t, &token, &token_size);

  // Check size
  if(token_size < 6)
  {
    return 0;
  }

  // Day
  date->day = Ascii_atoil(token + 0, 2);

  // Month
  date->month = Ascii_atoil(token + 2, 2);

  // year
  date->year = Ascii_atofl(token + 4, token_size - 4);

  return 1;
}

// Extract position
static uint8_t ExtractPosition(TokenIterate_t* t, NmeaPosition_t* pos)
{
  uint8_t* token;
  uint32_t token_size;

  int deg;
  float min;
  uint8_t dir = '-';

  // Get the token for lat
  TokenIterator_Forward(t, &token, &token_size);

  // Check size
  if (token_size < 4)
  {
    return 0;
  }

  // Get the degrees
  deg = Ascii_atoil(token, 2);

  // Get minutes
  min = Ascii_atofl(token + 2, token_size - 2);

  // Get direction
  if (!ExtractChar(t, &dir))
  {
    return 0;
  }

  pos->lat = (deg + (min / 60.0)) * ((dir == 'N') ? 1 : -1);

  // Get the token for lon
  TokenIterator_Forward(t, &token, &token_size);

  // Check size
  if (token_size < 4)
  {
    return 0;
  }

  // Get the degrees
  deg = Ascii_atoil(token, 3);

  // Get minutes
  min = Ascii_atofl(token + 3, token_size - 3);

  // Get direction
  if (!ExtractChar(t, &dir))
  {
    return 0;
  }

  pos->lon = (deg + (min / 60.0)) * ((dir == 'E') ? 1 : -1);

  return 1;
}


// Extract char
static uint8_t ExtractChar(TokenIterate_t* t, uint8_t* c)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIterator_Forward(t, &token, &token_size);

  // Check size
  if (token_size != 1)
  {
    *c = 0;
    return 0;
  }

  *c = token[0];

  return 1;
}

// Extract float
static uint8_t ExtractFloat(TokenIterate_t* t, float* x)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIterator_Forward(t, &token, &token_size);

  // Check size
  if (token_size == 0)
  {
    *x = 0;
    return 0;
  }

  *x = Ascii_atofl(token, token_size);

  return 1;
}

// Extract int
static uint8_t ExtractInt(TokenIterate_t* t, int32_t* x)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIterator_Forward(t, &token, &token_size);

  // Check size
  if (token_size == 0)
  {
    *x = 0;
    return 0;
  }

  *x = Ascii_atoil(token, token_size);

  return 1;
}

// Extract byte int
static uint8_t ExtractByteInt(TokenIterate_t* t, int8_t* x)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIterator_Forward(t, &token, &token_size);

  // Check size
  if (token_size == 0)
  {
    *x = 0;
    return 0;
  }

  *x = Ascii_atoil(token, token_size);

  return 1;
}

//// Sentence Parsers ////

static void ParseGsa(Nmea0183Instance_t* const inst, const uint32_t size)
{
  /*
    $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
    Where:
       GSA      Satellite status
       A        Auto selection of 2D or 3D fix (M = manual)
       3        3D fix - values include: 1 = no fix
                         2 = 2D fix
                         3 = 3D fix
       04,05... PRNs of satellites used for fix (space for 12)
       2.5      PDOP (dilution of precision)
       1.3      Horizontal dilution of precision (HDOP)
       2.1      Vertical dilution of precision (VDOP)
       *39      the checksum data, always begins with *
  */
}

static void ParseRmc(Nmea0183Instance_t* const inst, const uint32_t size)
{
  /*
  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   latitude 48 deg 07.038' N
     01131.000,E  longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *
  */

  GenericNmeaMessage_t msg;
  msg.message_type = NMEA_MESSAGE_TYPE_RMC;
  TokenIterate_t t;
  TokenIterator_Init(&t, ',', inst->packet_buffer + 6, size - 6);

  // Time
  ExtractTime(&t, &msg.rmc.time);

  // Fix
  ExtractChar(&t, &msg.rmc.fix);

  // Position
  ExtractPosition(&t, &msg.rmc.position);

  // Speed
  ExtractFloat(&t, &msg.rmc.speed);

  // Track
  ExtractFloat(&t, &msg.rmc.track);

  // Date
  ExtractDate(&t, &msg.rmc.date);

  // Try to queue it
  xQueueSendToBack(*inst->nmea_message_queue, &msg, 0);
}

static void ParseGga(Nmea0183Instance_t* const inst, const uint32_t size)
{
  /*
    $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    Where:
      GGA          Global Positioning System Fix Data
      123519       Fix taken at 12:35:19 UTC
      4807.038,N   latitude 48 deg 07.038' N
      01131.000,E  longitude 11 deg 31.000' E
      1            Fix quality: 0 = invalid
                  1 = GPS fix (SPS)
                  2 = DGPS fix
                  3 = PPS fix
            4 = Real Time Kinematic
            5 = Float RTK
                  6 = estimated (dead reckoning) (2.3 feature)
            7 = Manual input mode
            8 = Simulation mode
      08           Number of satellites being tracked
      0.9          Horizontal dilution of position
      545.4,M      Altitude, Meters, above mean sea level
      46.9,M       Height of geoid (mean sea level) above WGS84
              ellipsoid
      (empty field) time in seconds since last DGPS update
      (empty field) DGPS station ID number
      *47          the checksum data, always begins with *
  */

  GenericNmeaMessage_t msg;
  msg.message_type = NMEA_MESSAGE_TYPE_GGA;
  TokenIterate_t t;
  TokenIterator_Init(&t, ',', inst->packet_buffer + 6, size - 6);

  // Time
  ExtractTime(&t, &msg.gga.time);

  // Position
  ExtractPosition(&t, &msg.gga.position);

  // Fix
  ExtractChar(&t, &msg.gga.fix);

  // Sats tracked
  ExtractInt(&t, &msg.gga.sat_count);

  // HDOP
  ExtractFloat(&t, &msg.gga.h_dop);

  // Altitude
  ExtractFloat(&t, &msg.gga.altitude);

  // Altitude units
  ExtractChar(&t, &msg.gga.altitude_units);

  // Try to queue it
  xQueueSendToBack(*inst->nmea_message_queue, &msg, 0);
}

static void ParseZda(Nmea0183Instance_t* const inst, const uint32_t size)
{
  GenericNmeaMessage_t msg;
  msg.message_type = NMEA_MESSAGE_TYPE_ZDA;
  TokenIterate_t t;
  TokenIterator_Init(&t, ',', inst->packet_buffer + 6, size - 6);

  msg.zda.valid = true;

  // Time
  if (!ExtractTime(&t, &msg.zda.time))
  {
    msg.zda.valid = false;
  }

  // Day
  if (!ExtractByteInt(&t, (int8_t*)&msg.zda.day))
  {
    msg.zda.valid = false;
  }

  // Month
  if (!ExtractByteInt(&t, (int8_t*)&msg.zda.month))
  {
    msg.zda.valid = false;
  }

  // Year
  if (!ExtractInt(&t, (int32_t*)&msg.zda.year))
  {
    msg.zda.valid = false;
  }

  // Try to queue it
  xQueueSendToBack(*inst->nmea_message_queue, &msg, 0);
}
