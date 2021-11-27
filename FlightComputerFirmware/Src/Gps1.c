#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f7xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Spektrum.h"
#include "BigEndian.h"
#include "Ticks.h"
#include "Nmea0183.h"
#include "SerialInterface.h"
#include "Serial1.h"
#include "QueueBuffer.h"
#include "Log.h"

static Nmea0183Instance_t nmea_instance;
static QueueHandle_t nmea_message_queue;

#define GPS1_PERIOD   (10 / portTICK_PERIOD_MS)

static void Gps1Task(void* arg);

static const char* TAG = "GPS1";

void Gps1_Init(void)
{
  SerialInterface_t* serial_interface;

  nmea_message_queue = xQueueCreate(10, sizeof(GenericNmeaMessage_t));

  serial_interface = Serial1_GetInterface();

  nmea_instance.nmea_message_queue = &nmea_message_queue;
  nmea_instance.nmea_queue = serial_interface->rx_queue;

  serial_interface->init(9600, 0);

  Nmea0183_Init(&nmea_instance);

  xTaskCreate(Gps1Task, TAG, 2048, NULL, 0, NULL);
}

static void Gps1Task(void* arg)
{
  GenericNmeaMessage_t new_message;

  while (true)
  {
    Nmea0183_Process(&nmea_instance);

    while (xQueueReceive(*nmea_instance.nmea_message_queue, &new_message, 0) == pdTRUE)
    {
      switch (new_message.message_type)
      {
        case NMEA_MESSAGE_TYPE_GGA:
          printf("NMEA_MESSAGE_TYPE_GGA\r\n");
          break;

        case NMEA_MESSAGE_TYPE_RMC:
          printf("NMEA_MESSAGE_TYPE_RMA\r\n");
          break;

        case NMEA_MESSAGE_TYPE_GSA:
          printf("NMEA_MESSAGE_TYPE_GSA\r\n");
          break;

        case NMEA_MESSAGE_TYPE_ZDA:
          printf("NMEA_MESSAGE_TYPE_ZDA\r\n");
          break;

        default:
          LOG_W(TAG, "Unknown msg: %i", new_message.message_type);
          break;
      }
    }

    vTaskDelay(GPS1_PERIOD);
  }
}