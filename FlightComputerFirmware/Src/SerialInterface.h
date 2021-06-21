#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "event_groups.h"
#include "QueueBuffer.h"

enum SERIAL_INTERFACE_EVENT
{
  SERIAL_INTERFACE_EVENT_NEW_DATA_RX = (1 << 0),
};

typedef struct
{
  bool (*init)(const uint32_t baud, const uint32_t data_rx_event_count);
  void (*deinit)(void);
  QueueBuffer_t* tx_queue;
  QueueBuffer_t* rx_queue;
  void (*flush_tx)(void);
  EventGroupHandle_t* serial_events;
} SerialInterface_t;

#endif
