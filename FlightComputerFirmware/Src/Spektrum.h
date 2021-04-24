#ifndef SPEKTRUM_H
#define SPEKTRUM_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "SerialInterface.h"
#include "QueueBuffer.h"

#define SPEKTRUM_CHANNEL_COUNT 12

typedef struct
{
  bool valid;
  TickType_t timestamp;
  uint16_t value;
} SpektrumChannel_t;

typedef struct
{
  uint8_t fades;
  SpektrumChannel_t channels[SPEKTRUM_CHANNEL_COUNT];
} SpektrumFrame_t;

typedef struct
{
  void (*init_data_pin_bind)(void);
  void (*toggle_data_pin_bind)(const bool);
  void (*toggle_power_pin)(const bool);
  SerialInterface_t* serial_interface;
  TickType_t last_spektrum_frame_time;
  bool spektrum_timed_out;
} SpektrumInstance_t;

void Spektrum_Init(SpektrumInstance_t* const inst);
void Spektrum_Bind(SpektrumInstance_t* const inst);
uint32_t Spektrum_Process(SpektrumInstance_t* const inst, SpektrumFrame_t* const frames, const uint32_t max_frames);

#endif
