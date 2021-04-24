#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "QueueBuffer.h"

typedef struct
{
  bool (*init)(void);
  void (*deinit)(void);
  QueueBuffer_t* tx_queue;
  QueueBuffer_t* rx_queue;
  void (*flush_tx)(void);
} SerialInterface_t;

#endif
