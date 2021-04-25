#ifndef SPEKTRUMRCIN_H
#define SPEKTRUMRCIN_H

#include <stdbool.h>
#include <stdint.h>
#include "Spektrum.h"

typedef struct
{
  bool valid;
  TickType_t timestamp;
  float value;
} SpektrumRcInChannel_t;

typedef struct
{
  TickType_t timestamp;
  SpektrumRcInChannel_t channels[SPEKTRUM_CHANNEL_COUNT];
} SpektrumRcInStatus_t;

void SpectrumRcIn_Init(void);
bool SpectrumRcIn_Bind(void);
bool SpectrumRcIn_Reset(void);
bool SpectrumRcIn_GetStatus(SpektrumRcInStatus_t* const status);

#endif
