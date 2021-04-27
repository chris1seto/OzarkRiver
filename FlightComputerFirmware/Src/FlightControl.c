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

enum FLIGHTCONTROL_MODE
{
  FLIGHTCONTROL_MODE_DIRECT,
  FLIGHTCONTROL_MODE_RATE,
  FLIGHTCONTROL_MODE_ATTITUDE,
};

static enum FLIGHTCONTROL_MODE flight_control_mode = FLIGHTCONTROL_MODE_DIRECT;

enum FLIGHTCONTROL_FAULT
{
  FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID = (1 << 0),
};

enum RC_INPUT_CHANNEL
{
  RC_INPUT_CHANNEL_THROTTLE = 0,
  RC_INPUT_CHANNEL_AILERON = 1,
  RC_INPUT_CHANNEL_ELEVATOR = 2,
  RC_INPUT_CHANNEL_RUDDER = 3,
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
static const ServoActuatorTranslation_t servo_accuator_translations[SERVO_ACTUATOR_COUNT] =
{
  [SERVO_ACTUATOR_OUTPUT_LEFT_ELEVON]  = {1.1f, 1.65f, 2.0f, SERVOOUT_CHANNEL_1},
  [SERVO_ACTUATOR_OUTPUT_RIGHT_ELEVON] = {2.0f, 1.40f, 1.1f, SERVOOUT_CHANNEL_2},
  [SERVO_ACTUATOR_OUTPUT_THROTTLE] =     {1.0f, 1.5f, 2.0f, SERVOOUT_CHANNEL_3},
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
  xTaskCreate(FlightControlTask, TAG, 512, NULL, 0, NULL);
}

static bool SpektrumChannelValid(const SpektrumRcInChannel_t* channel)
{
  if (!channel->valid || Ticks_IsExpired(channel->timestamp, 100))
  {
    return false;
  }

  return true;
}

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
      control_outputs.throttle = CONTROL_MIN;
    }
    else
    {
      control_outputs.yaw = 0;
      control_outputs.pitch = spektrum_status.channels[RC_INPUT_CHANNEL_ELEVATOR].value;
      control_outputs.roll = spektrum_status.channels[RC_INPUT_CHANNEL_AILERON].value;
      control_outputs.throttle = spektrum_status.channels[RC_INPUT_CHANNEL_THROTTLE].value;
    }

    CommitActuators(&control_outputs);

    vTaskDelay(FLIGHTCONTROL_PERIOD);
  }
}

static void ServoActuatorTranslate(const float* input, const ServoActuatorTranslation_t* translation, const uint32_t translation_count)
{
  uint32_t i;
  float channel_output;
  float channel_constrain;
  float output[3];

  for (i = 0; i < translation_count; i++)
  {
    channel_constrain = MathX_ConstrainF(input[i], CONTROL_MIN, CONTROL_MAX);

    if (input[i] >= 0)
    {
      channel_output = MathX_MapF(channel_constrain, 0, CONTROL_MAX, translation[i].output_center, translation[i].output_max);
    }
    else
    {
      channel_output = MathX_MapF(channel_constrain, 0, CONTROL_MIN, translation[i].output_center, translation[i].output_min);
    }

    output[i] = channel_output;
    ServoOut_Set(translation[i].target, channel_output);
  }
}

static void CommitActuators(const FlightControlOutput_t* controls)
{
  float actuators[SERVO_ACTUATOR_COUNT];

  actuators[SERVO_ACTUATOR_OUTPUT_LEFT_ELEVON] = -controls->pitch - controls->roll;
  actuators[SERVO_ACTUATOR_OUTPUT_RIGHT_ELEVON] = -controls->pitch + controls->roll;
  actuators[SERVO_ACTUATOR_OUTPUT_THROTTLE] = controls->throttle;

  ServoActuatorTranslate((float*)&actuators, (ServoActuatorTranslation_t*)&servo_accuator_translations, SERVO_ACTUATOR_COUNT);
}
