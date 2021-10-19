#ifndef CRSF_H
#define CRSF_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#define CRSF_CHANNEL_COUNT 16

typedef struct
{
  TickType_t timestamp;
  float channels[CRSF_CHANNEL_COUNT];
} CrsfChannelData_t;

typedef struct
{
  TickType_t timestamp;
  uint8_t uplink_rssi_1;
  uint8_t uplink_rssi_2;
  uint8_t uplink_link_quality;
  int8_t uplink_snr;
  uint8_t active_antenna;
  uint8_t rf_mode;
  uint8_t uplink_tx_power;
  uint8_t downlink_rssi;
  uint8_t downlink_link_quality;
  int8_t downlink_snr;
} CrsfLinkStatistics_t;

typedef struct
{
  CrsfChannelData_t channel_data;
  CrsfLinkStatistics_t link_statistics;
} CrsfStatus_t;

void Crsf_Init(void);
bool Crsf_GetStatus(CrsfStatus_t* const status);

#endif