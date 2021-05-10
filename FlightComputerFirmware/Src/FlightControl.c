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
#include "Pid.h"
#include "ImuAhrs.h"

#define CONTROL_MIN -1000.0f
#define CONTROL_MAX 1000.0f
#define CONTROL_RANGE (CONTROL_MAX * 2)

static Pid_t pitch_rate_pid = {0};
static Pid_t roll_rate_pid = {0};

enum FLIGHTCONTROL_MODE
{
  FLIGHTCONTROL_MODE_DIRECT,
  FLIGHTCONTROL_MODE_RATE,
  FLIGHTCONTROL_MODE_ATTITUDE,
};

enum FLIGHTCONTROL_COMMAND
{
  FLIGHTCONTROL_COMMAND_NONE,
  FLIGHTCONTROL_COMMAND_DIRECT,
  FLIGHTCONTROL_COMMAND_RATE,
  FLIGHTCONTROL_COMMAND_ATTITUDE,
};

typedef struct
{
  float pitch;
  float roll;
  float yaw;
} FlightControlCommandDirect_t;

typedef struct
{
  float pitch_rate;
  float roll_rate;
  float yaw_rate;
} FlightControlCommandRate_t;

typedef struct
{
  float pitch;
  float roll;
  float yaw;
} FlightControlCommandAttitude_t;

typedef struct
{
  enum FLIGHTCONTROL_COMMAND command;

  union
  {
    FlightControlCommandDirect_t direct;
    FlightControlCommandRate_t rate;
    FlightControlCommandAttitude_t attitude;
  };

  float throttle;
} FlightControlCommand_t;

enum FLIGHTCONTROL_FAULT
{
  FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID = (1 << 0),
  FLIGHTCONTROL_FAULT_IMUAHRS_INVALID = (1 << 1),
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

#define PITCH_RATE_STICK_SCALER .3f
#define ROLL_RATE_STICK_SCALER .5f

static enum FLIGHTCONTROL_COMMAND last_flight_control_command = FLIGHTCONTROL_COMMAND_NONE;

static bool SpektrumChannelValid(const SpektrumRcInChannel_t* channel);
static void ServoActuatorTranslate(const float* input, const ServoActuatorTranslation_t* translation, const uint32_t translation_count);
static void CommitActuators(const FlightControlOutput_t* controls);
static void FlightControlTask(void* arg);

static const char* TAG = "FLIGHTCONTROL";

#define FLIGHTCONTROL_PERIOD   (10 / portTICK_PERIOD_MS)

void FlightControl_Init(void)
{
  // Configure PID
  Pid_SetOutputLimit(&pitch_rate_pid, CONTROL_MIN, CONTROL_MAX);
  Pid_SetOutputLimit(&roll_rate_pid, CONTROL_MIN, CONTROL_MAX);

  Pid_SetTuning(&pitch_rate_pid, FLIGHTCONTROL_PERIOD, 2.5, 0, 0);
  Pid_SetTuning(&roll_rate_pid, FLIGHTCONTROL_PERIOD, 2.5, 0, 0);

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
  ImuAhrsStatus_t imu_ahrs_status = {0};
  uint32_t faults;
  enum FLIGHTCONTROL_MODE tx_flight_control_mode;
  FlightControlCommand_t flight_control_command;

  while (true)
  {
    // Clear faults
    faults = 0;

    // Get ImuAhrs status
    if (!ImuAhrs_GetStatus(&imu_ahrs_status))
    {
      Bits_Set(&faults, FLIGHTCONTROL_FAULT_IMUAHRS_INVALID);
    }
    else
    {
      // Check if there is any condition that would make the imuahrs unusable
      if (Bits_IsSet(imu_ahrs_status.flags, IMUAHRS_FLAGS_MPU6050_FAIL)
        || Bits_IsSet(imu_ahrs_status.flags, IMUAHRS_FLAGS_HMC5983_FAIL)
        || Bits_IsSet(imu_ahrs_status.flags, IMUAHRS_FLAGS_CALIBRATING))
      {
        Bits_Set(&faults, FLIGHTCONTROL_FAULT_IMUAHRS_INVALID);
      }
    }

    // Get Spektrum status
    if (!SpectrumRcIn_GetStatus(&spektrum_status))
    {
      Bits_Set(&faults, FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID);
    }
    else
    {
      if (!SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_THROTTLE])
        || !SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_RUDDER])
        || !SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_AILERON])
        || !SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_ELEVATOR])
        || !SpektrumChannelValid(&spektrum_status.channels[RC_INPUT_CHANNEL_MODE]))
      {
        Bits_Set(&faults, FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID);
      }
    }

    // Select flight mode
    if (spektrum_status.channels[RC_INPUT_CHANNEL_MODE].value > 500)
    {
      tx_flight_control_mode = FLIGHTCONTROL_MODE_ATTITUDE;
    }
    else if (spektrum_status.channels[RC_INPUT_CHANNEL_MODE].value > -500)
    {
      tx_flight_control_mode = FLIGHTCONTROL_MODE_RATE;
    }
    else
    {
      tx_flight_control_mode = FLIGHTCONTROL_MODE_DIRECT;
    }

    // Check if we have TX input
    if (Bits_IsSet(faults, FLIGHTCONTROL_FAULT_SPEKTRUM_INVALID))
    {
      flight_control_command.command = FLIGHTCONTROL_COMMAND_DIRECT;
      flight_control_command.direct.pitch = 0;
      flight_control_command.direct.roll = 0;
      flight_control_command.direct.yaw = 0;
      flight_control_command.throttle = 0;
    }
    else
    {
      if (!Bits_IsSet(faults, FLIGHTCONTROL_FAULT_IMUAHRS_INVALID))
      {
        tx_flight_control_mode = FLIGHTCONTROL_MODE_DIRECT;
      }

      switch (tx_flight_control_mode)
      {
        case FLIGHTCONTROL_MODE_ATTITUDE:
        case FLIGHTCONTROL_MODE_DIRECT:
          flight_control_command.command = FLIGHTCONTROL_COMMAND_DIRECT;
          flight_control_command.direct.pitch = spektrum_status.channels[RC_INPUT_CHANNEL_ELEVATOR].value;
          flight_control_command.direct.roll = spektrum_status.channels[RC_INPUT_CHANNEL_AILERON].value;
          flight_control_command.direct.yaw = 0;
          flight_control_command.throttle = spektrum_status.channels[RC_INPUT_CHANNEL_THROTTLE].value;
          break;

        case FLIGHTCONTROL_MODE_RATE:
          flight_control_command.command = FLIGHTCONTROL_COMMAND_RATE;
          flight_control_command.rate.pitch_rate = spektrum_status.channels[RC_INPUT_CHANNEL_ELEVATOR].value * PITCH_RATE_STICK_SCALER;
          flight_control_command.rate.roll_rate = spektrum_status.channels[RC_INPUT_CHANNEL_AILERON].value * ROLL_RATE_STICK_SCALER;
          flight_control_command.rate.yaw_rate = 0;
          flight_control_command.throttle = spektrum_status.channels[RC_INPUT_CHANNEL_THROTTLE].value;
          break;

        /*case FLIGHTCONTROL_MODE_ATTITUDE:
          flight_control_command.command = FLIGHTCONTROL_COMMAND_RATE;
          flight_control_command.rate.pitch = 0;
          flight_control_command.rate.roll = 0;
          flight_control_command.rate.yaw = 0;
          flight_control_command.throttle = 0;
          break;*/
      }
    }

    if (flight_control_command.command != last_flight_control_command)
    {
      // Reset
      Pid_Reset(&pitch_rate_pid);
      Pid_Reset(&roll_rate_pid);
    }

    switch (flight_control_command.command)
    {
      // Direct pass through
      case FLIGHTCONTROL_COMMAND_DIRECT:
        control_outputs.yaw = flight_control_command.direct.yaw;
        control_outputs.pitch = flight_control_command.direct.pitch;
        control_outputs.roll = flight_control_command.direct.roll;
        control_outputs.throttle = flight_control_command.throttle;
        last_flight_control_command =  FLIGHTCONTROL_COMMAND_DIRECT;
        break;

      // Rate based (degrees per second)
      case FLIGHTCONTROL_COMMAND_RATE:
        control_outputs.yaw = 0;
        control_outputs.pitch = Pid_Calculate(&pitch_rate_pid, flight_control_command.rate.pitch_rate, imu_ahrs_status.pitch_rate);
        control_outputs.roll = Pid_Calculate(&roll_rate_pid, flight_control_command.rate.roll_rate, imu_ahrs_status.roll_rate);
        control_outputs.throttle = flight_control_command.throttle;
        last_flight_control_command = FLIGHTCONTROL_COMMAND_RATE;
        break;

      case FLIGHTCONTROL_COMMAND_ATTITUDE:
        last_flight_control_command = FLIGHTCONTROL_COMMAND_ATTITUDE;
        break;

      default:
        break;
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
