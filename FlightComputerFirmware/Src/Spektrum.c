#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Spektrum.h"
#include "Ticks.h"

#define MASK_2048_CHANID 0x000F
#define MASK_2048_SXPOS  0x07FF

#define SPEKTRUM_SYSTEM_FIELD_DSMX11 0xb2
#define SPEKTRUM_CHANNELS_PER_FRAME  7
#define SPEKTRUM_FRAME_SIZE          2 + (2 * SPEKTRUM_CHANNELS_PER_FRAME)

#define SPEKTRUM_TICKS_TO_NORMAL(x) ((x * 0.9765625f) - 1000.0f)

void Spektrum_Init(SpektrumInstance_t* const inst)
{
  // Toggle Spektrum power on
  if (inst->toggle_power_pin != NULL)
  {
    inst->toggle_power_pin(true);
  }

  // Init serial
  if (inst->serial_interface != NULL)
  {
    inst->serial_interface->init(115200, 0);
  }
}

void Spektrum_Bind(SpektrumInstance_t* const inst)
{
  uint32_t i;

  // Deinit serial
  inst->serial_interface->deinit();

  // Init GPIO bind pin
  inst->init_data_pin_bind();

  // Toggle Spektrum power off
  inst->toggle_power_pin(false);
  inst->toggle_data_pin_bind(true);

  HAL_Delay(1000);

  // Toggle Spektrum power on
  inst->toggle_power_pin(true);

  HAL_Delay(100);

  for (i = 0; i < 9; i++)
  {
    inst->toggle_data_pin_bind(false);
    HAL_Delay(10);
    inst->toggle_data_pin_bind(true);
    HAL_Delay(10);
  }

  // Init serial again
  inst->serial_interface->init(115200, 0);
}

uint32_t Spektrum_Process(SpektrumInstance_t* const inst, SpektrumFrame_t* const frames, const uint32_t max_frames)
{
  uint32_t i;
  uint8_t temp8;
  uint16_t temp16;
  SpektrumFrame_t new_frame;
  uint32_t frame_count = 0;
  uint16_t channel_id;
  uint16_t channel_value;
  bool phase;
  TickType_t time_now;

  while (QueueBuffer_Count(inst->serial_interface->rx_queue) > SPEKTRUM_FRAME_SIZE && frame_count < max_frames)
  {
    // Sync to the header word
    QueueBuffer_Peek(inst->serial_interface->rx_queue, 1, &temp8);

    // Try to get a sync/header byte
    if (temp8 != SPEKTRUM_SYSTEM_FIELD_DSMX11)
    {
      QueueBuffer_Dequeue(inst->serial_interface->rx_queue, 1);
      continue;
    }

    // Recover frame content
    time_now = Ticks_Now();

    // Clear current content in the frame
    for (i = 0; i < SPEKTRUM_CHANNEL_COUNT; i++)
    {
      frames[frame_count].channels[i].valid = false;
      frames[frame_count].channels[i].timestamp = 0;
      frames[frame_count].channels[i].value = 0;
    }

    // Fades
    QueueBuffer_Peek(inst->serial_interface->rx_queue, 0, &new_frame.fades);

    // Channel position
    for (i = 0; i < SPEKTRUM_CHANNELS_PER_FRAME; i++)
    {
      temp16 = 0;

      QueueBuffer_Peek(inst->serial_interface->rx_queue, 2 + i * 2, &temp8);
      temp16 |= temp8 << 8;

      QueueBuffer_Peek(inst->serial_interface->rx_queue, 2 + i * 2 + 1, &temp8);
      temp16 |= temp8;

      channel_id = (temp16 >> 11) & MASK_2048_CHANID;
      phase = temp16 >> 15;

      if (channel_id < SPEKTRUM_CHANNEL_COUNT)
      {
        channel_value = (temp16 >> 0) & MASK_2048_SXPOS;

        frames[frame_count].channels[channel_id].valid = true;
        frames[frame_count].channels[channel_id].timestamp = time_now;
        frames[frame_count].channels[channel_id].value = SPEKTRUM_TICKS_TO_NORMAL((float)channel_value);
      }
      else if (channel_id == SPEKTRUM_CHANNEL_COUNT)
      {
      }
    }

    QueueBuffer_Dequeue(inst->serial_interface->rx_queue, SPEKTRUM_FRAME_SIZE);

    frame_count++;
  }

  return frame_count;
}