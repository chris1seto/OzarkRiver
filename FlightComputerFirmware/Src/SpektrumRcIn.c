#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f3xx_hal.h>
#include "Ticks.h"
#include "Spektrum.h"
#include "Serial3.h"
#include "SpektrumRcIn.h"

static SpektrumInstance_t spektrum_rx;

static QueueHandle_t spektrum_status_queue;

static SpektrumRcInStatus_t spektrum_status = {0};

static void SpektrumRcInTask(void* arg);

#define NEW_SPEKTRUM_FRAME_COUNT 5

#define SPEKTRUMRCIN_PERIOD   (10 / portTICK_PERIOD_MS)
#define SPEKTRUM_TIMEOUT      (50 / portTICK_PERIOD_MS)

static const char* TAG = "SPEKTRUM";

void SpectrumRcIn_Init(void)
{
  SerialInterface_t* serial_interface;

  spektrum_status_queue = xQueueCreate(1, sizeof(SpektrumRcInStatus_t));

  serial_interface = Serial3_GetInterface();

  spektrum_rx.init_data_pin_bind = NULL;
  spektrum_rx.toggle_data_pin_bind = NULL;
  spektrum_rx.toggle_power_pin = NULL;
  spektrum_rx.serial_interface = serial_interface;

  Spektrum_Init(&spektrum_rx);

  //Spektrum_Bind(&spektrum_rx);

  xTaskCreate(SpektrumRcInTask, TAG, 256, NULL, 0, NULL);
}

bool SpectrumRcIn_Bind(void)
{
  return true;
}

bool SpectrumRcIn_Reset(void)
{
  return true;
}

bool SpectrumRcIn_GetStatus(SpektrumRcInStatus_t* const status)
{
  return (xQueueReceive(spektrum_status_queue, status, 0) == pdTRUE);
}

static void SpektrumRcInTask(void* arg)
{
  SpektrumFrame_t new_spektrum_frames[NEW_SPEKTRUM_FRAME_COUNT] = {0};
  uint32_t recovered_frames;
  uint32_t i;
  uint32_t i_channel;

  while (true)
  {
    recovered_frames = Spektrum_Process(&spektrum_rx, (SpektrumFrame_t*)&new_spektrum_frames, NEW_SPEKTRUM_FRAME_COUNT);

    // For each recovered frame...
    for (i = 0; i < recovered_frames; i++)
    {
      // ... For each channel, merge into the main status
      for (i_channel = 0; i_channel < SPEKTRUM_CHANNEL_COUNT; i_channel++)
      {
        // If the channel is valid and not timed out, merge it in
        if (new_spektrum_frames[i].channels[i_channel].valid
          && !Ticks_IsExpired(new_spektrum_frames[i].channels[i_channel].timestamp, SPEKTRUM_TIMEOUT))
        {
          spektrum_status.channels[i_channel].valid = true;
          spektrum_status.channels[i_channel].timestamp = new_spektrum_frames[i].channels[i_channel].timestamp;
          spektrum_status.channels[i_channel].value = new_spektrum_frames[i].channels[i_channel].value;
        }
      }
    }

    // Check for channel timeouts
    for (i_channel = 0; i_channel < SPEKTRUM_CHANNEL_COUNT; i_channel++)
    {
      if (Ticks_IsExpired(spektrum_status.channels[i_channel].timestamp, SPEKTRUM_TIMEOUT))
      {
        spektrum_status.channels[i_channel].valid = false;
        spektrum_status.channels[i_channel].timestamp = 0;
        spektrum_status.channels[i_channel].value = 0;
      }
    }
    
    spektrum_status.timestamp = Ticks_Now();

    xQueueSendToBack(spektrum_status_queue, &spektrum_status, 0);

    vTaskDelay(SPEKTRUMRCIN_PERIOD);
  }
}