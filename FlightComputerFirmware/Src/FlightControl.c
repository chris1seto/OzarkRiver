#include <stdio.h>
#include <stdbool.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Retarget.h"
#include "Leds.h"
#include "SpektrumRcIn.h"
#include "ServoOut.h"
#include "Bits.h"
#include "MathX.h"
#include "Ticks.h"

#define CONTROL_MIN -1000.0f
#define CONTROL_MAX 1000.0f

enum RC_INPUT_CHANNEL
{
  RC_INPUT_CHANNEL_THROTTLE = 0,
  RC_INPUT_CHANNEL_RUDDER = 1,
  RC_INPUT_CHANNEL_AILERON = 2,
  RC_INPUT_CHANNEL_ELEVATOR = 3,
  RC_INPUT_CHANNEL_MODE = 4,
};

#define RC_INPUT_CHANNEL_COUNT 5

enum SERVO_ACTUATOR_OUTPUT
{
  SERVO_ACTUATOR_OUTPUT_LEFT_ELEVON = 0,
  SERVO_ACTUATOR_OUTPUT_RIGHT_ELEVON = 1,
  SERVO_ACTUATOR_OUTPUT_THROTTLE = 2,
};

typedef struct
{
  float output_min;
  float output_center;
  float output_max;
  enum SERVOOUT_CHANNEL target;
} ServoActuatorTranslation_t;

#define SERVO_ACTUATOR_COUNT 3
const ServoActuatorTranslation_t servo_accuator_translations[SERVO_ACTUATOR_COUNT] =
{
  [SERVO_ACTUATOR_OUTPUT_LEFT_ELEVON]  = {1400.0f, 1500.0f, 1600.0f, SERVO_ACTUATOR_OUTPUT_LEFT_ELEVON},
  [SERVO_ACTUATOR_OUTPUT_RIGHT_ELEVON] = {1400.0f, 1500.0f, 1600.0f, SERVO_ACTUATOR_OUTPUT_RIGHT_ELEVON},
  [SERVO_ACTUATOR_OUTPUT_THROTTLE] =     {1000.0f, 1500.0f, 2000.0f, SERVO_ACTUATOR_OUTPUT_THROTTLE},
};

typedef struct
{
  float pitch;
  float roll;
  float yaw;
  float throttle;
} FlightControlOutput_t;

static bool SpektrumChannelValid(const SpektrumRcInChannel_t* channel);
static void ServoActuatorTranslate(const float* input, const ServoActuatorTranslation_t* translation, const uint32_t translation_count);
static void CommitActuators(const FlightControlOutput_t* controls);
static void FlightControlTask(void* arg);

static const char* TAG = "FLIGHTCONTROL";

#define FLIGHTCONTROL_PERIOD   (20 / portTICK_PERIOD_MS)

void FlightControl_Init(void)
{
  xTaskCreate(FlightControlTask, TAG, 256, NULL, 0, NULL);
}

static bool SpektrumChannelValid(const SpektrumRcInChannel_t* channel)
{
  if (!channel->valid || Ticks_IsExpired(channel->timestamp, 100))
  {
    return false;
  }

  return true;
}

enum FLIGHTCONTROL_FAULT
{
  FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID = (1 << 0),
};

static void FlightControlTask(void* arg)
{
  SpektrumRcInStatus_t spektrum_status = {0};
  FlightControlOutput_t control_outputs = {0};
  uint32_t faults;

  while (true)
  {
    // Clear faults
    faults = 0;

    // Get Spektrum status
    if (!SpectrumRcIn_GetStatus(&spektrum_status))
    {
      SET(faults, FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID);
    }
    else
    {
      if (!SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_THROTTLE])
        || !SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_RUDDER])
        || !SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_AILERON])
        || !SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_ELEVATOR])
        || !SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_MODE]))
      {
        SET(faults, FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID);
      }
    }

    if (IS_SET(faults, FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID))
    {
      control_outputs.yaw = 0;
      control_outputs.pitch = 0;
      control_outputs.roll = 0;
      control_outputs.throttle = 0;
    }
    else
    {
      control_outputs.yaw = 0;
      control_outputs.pitch = spektrum_status.channels[RC_INPUT_CHANNEL_ELEVATOR];
      control_outputs.roll = spektrum_status.channels[RC_INPUT_CHANNEL_AILERON];
      control_outputs.throttle = spektrum_status.channels[RC_INPUT_CHANNEL_THROTTLE];
    }

    CommitActuators(&control_outputs);

    vTaskDelay(FLIGHTCONTROL_PERIOD);
  }
}

static void ServoActuatorTranslate(const float* input, const ServoActuatorTranslation_t* translation, const uint32_t translation_count)
{
  uint32_t i;
  float channel_output;
  float channel_input;

  for (i = 0; i < translation_count; i++)
  {
    channel_input = MathX_ConstrainF(input, CONTROL_MIN, CONTROL_MAX);

    if (input[i] >= 0)
    {
    }
    else
    {
    }

    //ServoOut_Set(translation[i].target, channel_output);
  }
}

static void CommitActuators(const FlightControlOutput_t* controls)
{
  float actuators[SERVO_ACTUATOR_COUNT];

  actuators[SERVO_ACTUATOR_OUTPUT_LEFT_ELEVON] = controls->pitch + controls->roll;
  actuators[SERVO_ACTUATOR_OUTPUT_RIGHT_ELEVON] = controls->pitch - controls->roll;
  actuators[SERVO_ACTUATOR_OUTPUT_THROTTLE] = controls->throttle;

  ServoActuatorTranslate(&actuators, &servo_accuator_translations, SERVO_ACTUATOR_COUNT);
}
