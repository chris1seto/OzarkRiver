#include <stdio.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Retarget.h"
#include "Leds.h"
#include "Crsf.h"
#include "ServoOut.h"
#include "Bits.h"
#include "MathX.h"
#include "Ticks.h"
#include "Pid.h"
#include "ImuAhrs.h"

#define CONTROL_MIN -1000.0f
#define CONTROL_MAX 1000.0f
#define CONTROL_RANGE (CONTROL_MAX * 2)

#define PITCH_RATE_MAX    120
#define ROLL_RATE_MAX     120

static Pid_t pitch_rate_pid = {0};
static Pid_t roll_rate_pid = {0};

static Pid_t pitch_attitude_pid = {0};
static Pid_t roll_attitude_pid = {0};

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
  float throttle;
  float flap;
} FlightControlCommandDirect_t;

typedef struct
{
  float pitch_rate;
  float roll_rate;
  float yaw_rate;
  float throttle;
  float flap;
} FlightControlCommandRate_t;

typedef struct
{
  float pitch;
  float roll;
  float yaw;
  float throttle;
  float flap;
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

} FlightControlCommand_t;

enum FLIGHTCONTROL_FAULT
{
  FLIGHTCONTROL_FAULT_RCIN_INVALID = (1 << 0),
  FLIGHTCONTROL_FAULT_IMUAHRS_INVALID = (1 << 1),
};

enum RC_INPUT_CHANNEL
{
  RC_INPUT_CHANNEL_THROTTLE = 0,
  RC_INPUT_CHANNEL_AILERON = 1,
  RC_INPUT_CHANNEL_ELEVATOR = 2,
  RC_INPUT_CHANNEL_RUDDER = 3,
  RC_INPUT_CHANNEL_ARM = 4,
  RC_INPUT_CHANNEL_MODE = 5,
  RC_INPUT_CHANNEL_FLAPS = 6,
};

#define RC_INPUT_CHANNEL_COUNT 6

enum SERVO_ACTUATOR_OUTPUT
{
  SERVO_ACTUATOR_OUTPUT_THROTTLE = 0,
  SERVO_ACTUATOR_OUTPUT_ELEVATOR = 1,
  SERVO_ACTUATOR_OUTPUT_RUDDER = 2,
  SERVO_ACTUATOR_OUTPUT_AILERON_LEFT = 3,
  SERVO_ACTUATOR_OUTPUT_AILERON_RIGHT = 4,
};

typedef struct
{
  float output_min;
  float output_center;
  float output_max;
  enum SERVOOUT_CHANNEL target;
} ServoActuatorTranslation_t;

#define SERVO_ACTUATOR_COUNT 5
static const ServoActuatorTranslation_t servo_accuator_translations[SERVO_ACTUATOR_COUNT] =
{
  [SERVO_ACTUATOR_OUTPUT_THROTTLE]        = {1.0f, 1.5f, 2.0f, SERVOOUT_CHANNEL_1},
  [SERVO_ACTUATOR_OUTPUT_ELEVATOR]        = {2.0f, 1.5f, 1.0f, SERVOOUT_CHANNEL_2},
  [SERVO_ACTUATOR_OUTPUT_RUDDER]          = {2.0f, 1.5f, 1.0f, SERVOOUT_CHANNEL_3},
  //                                          D       C      U
  [SERVO_ACTUATOR_OUTPUT_AILERON_LEFT]    = {0.85f, 1.65f, 2.40f, SERVOOUT_CHANNEL_4},
  //                                          U       C      D
  [SERVO_ACTUATOR_OUTPUT_AILERON_RIGHT]   = {2.60f, 1.65f, 1.00f, SERVOOUT_CHANNEL_5},
};

typedef struct
{
  float pitch;
  float roll;
  float yaw;
  float throttle;
  float flap;
} FlightControlOutput_t;

#define PITCH_RATE_STICK_SCALER .45f
#define ROLL_RATE_STICK_SCALER  .5f

#define PITCH_ATTITUDE_STICK_SCALER (90.0f / CONTROL_MAX)
#define ROLL_ATTITUDE_STICK_SCALER (90.0f / CONTROL_MAX)

#define RATE_INTEGRATOR_MAX 250

#define FLAP_SCALER .5f

static enum FLIGHTCONTROL_COMMAND last_flight_control_command = FLIGHTCONTROL_COMMAND_NONE;

static void ServoActuatorTranslate(const float* input, const ServoActuatorTranslation_t* translation, const uint32_t translation_count);
static void CommitActuators(const FlightControlOutput_t* controls);
static void FlightControlTask(void* arg);

static const char* TAG = "FLIGHTCONTROL";

#define FLIGHTCONTROL_PERIOD   (10 / portTICK_PERIOD_MS)
#define RCINPUT_TIMEOUT   (250 / portTICK_PERIOD_MS)

void FlightControl_Init(void)
{
  // Configure rate PID
  Pid_SetOutputLimit(&pitch_rate_pid, CONTROL_MIN, CONTROL_MAX);
  Pid_SetOutputLimit(&roll_rate_pid, CONTROL_MIN, CONTROL_MAX);

  Pid_SetTuning(&pitch_rate_pid, FLIGHTCONTROL_PERIOD, 2.4, .5, 0);
  Pid_SetTuning(&roll_rate_pid, FLIGHTCONTROL_PERIOD, 1.5, .25, 0);

  Pid_SetIntegratorError(&pitch_rate_pid, -RATE_INTEGRATOR_MAX, RATE_INTEGRATOR_MAX);
  Pid_SetIntegratorError(&roll_rate_pid, -RATE_INTEGRATOR_MAX, RATE_INTEGRATOR_MAX);

  // Configure attitude PID
  Pid_SetOutputLimit(&pitch_attitude_pid, -PITCH_RATE_MAX, PITCH_RATE_MAX);
  Pid_SetOutputLimit(&roll_attitude_pid, -ROLL_RATE_MAX, ROLL_RATE_MAX);

  Pid_SetTuning(&pitch_attitude_pid, FLIGHTCONTROL_PERIOD, 1.9, 0, 0);
  Pid_SetTuning(&roll_attitude_pid, FLIGHTCONTROL_PERIOD, 2.0, 0, 0);

  xTaskCreate(FlightControlTask, TAG, 600, NULL, 0, NULL);
}

static void ComputeAttitude(FlightControlOutput_t* const control_outputs, const ImuAhrsStatus_t* imu_ahrs_status, const FlightControlCommandAttitude_t* command)
{
  float pitch_rate_setpoint;
  float roll_rate_setpoint;

  float pitch_rate_correction;
  float roll_rate_correction;

  // Compute attitude -> rate correction
  pitch_rate_setpoint = Pid_Calculate(&pitch_attitude_pid, command->pitch, imu_ahrs_status->pitch);
  roll_rate_setpoint = Pid_Calculate(&roll_attitude_pid, command->roll, imu_ahrs_status->roll);

  // Compute rate correction
  pitch_rate_correction = Pid_Calculate(&pitch_rate_pid, pitch_rate_setpoint, imu_ahrs_status->pitch_rate);
  roll_rate_correction = Pid_Calculate(&roll_rate_pid, roll_rate_setpoint, imu_ahrs_status->roll_rate);

  /*printf("att (%+.2f %+.2f)  rat (%+.2f %+.2f)  setpoint (%+.2f %+.2f) out(%+.2f %+.2f)\r\n",
    imu_ahrs_status->pitch, imu_ahrs_status->roll,
    imu_ahrs_status->pitch_rate, imu_ahrs_status->roll_rate,
    pitch_rate_setpoint, roll_rate_setpoint,
    pitch_rate_correction, roll_rate_correction);*/

  control_outputs->yaw = 0;
  control_outputs->pitch = pitch_rate_correction;
  control_outputs->roll = roll_rate_correction;
  control_outputs->throttle = command->throttle;
}

static void FlightControlTask(void* arg)
{
  CrsfStatus_t crsf_status = {0};
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

    // Get Crsf status
    if (!Crsf_GetStatus(&crsf_status))
    {
      Bits_Set(&faults, FLIGHTCONTROL_FAULT_RCIN_INVALID);
    }
    else
    {
      if (Ticks_IsExpired(crsf_status.channel_data.timestamp, RCINPUT_TIMEOUT))
      {
        Bits_Set(&faults, FLIGHTCONTROL_FAULT_RCIN_INVALID);
      }
    }

    // Select flight mode
    if (crsf_status.channel_data.channels[RC_INPUT_CHANNEL_MODE] > 500)
    {
      tx_flight_control_mode = FLIGHTCONTROL_MODE_RATE;
    }
    else if (crsf_status.channel_data.channels[RC_INPUT_CHANNEL_MODE] > -500)
    {
      tx_flight_control_mode = FLIGHTCONTROL_MODE_RATE;
    }
    else
    {
      tx_flight_control_mode = FLIGHTCONTROL_MODE_DIRECT;
    }

    // Check if we have TX input
    if (Bits_IsSet(faults, FLIGHTCONTROL_FAULT_RCIN_INVALID))
    {
      flight_control_command.command = FLIGHTCONTROL_COMMAND_DIRECT;
      flight_control_command.direct.pitch = 0;
      flight_control_command.direct.roll = 0;
      flight_control_command.direct.yaw = 0;
      flight_control_command.direct.throttle = CONTROL_MIN;
    }
    else
    {
      if (Bits_IsSet(faults, FLIGHTCONTROL_FAULT_IMUAHRS_INVALID))
      {
        tx_flight_control_mode = FLIGHTCONTROL_MODE_DIRECT;
      }

      switch (tx_flight_control_mode)
      {
        case FLIGHTCONTROL_MODE_DIRECT:
          flight_control_command.command = FLIGHTCONTROL_COMMAND_DIRECT;
          flight_control_command.direct.pitch = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_ELEVATOR];
          flight_control_command.direct.roll = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_AILERON];
          flight_control_command.direct.yaw = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_RUDDER];
          flight_control_command.direct.throttle = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_THROTTLE];
          flight_control_command.direct.flap = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_FLAPS];
          break;

        case FLIGHTCONTROL_MODE_RATE:
          flight_control_command.command = FLIGHTCONTROL_COMMAND_RATE;
          flight_control_command.rate.pitch_rate = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_ELEVATOR] * PITCH_RATE_STICK_SCALER;
          flight_control_command.rate.roll_rate = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_AILERON] * ROLL_RATE_STICK_SCALER;
          flight_control_command.rate.yaw_rate = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_RUDDER];
          flight_control_command.rate.throttle = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_THROTTLE];
          flight_control_command.rate.flap = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_FLAPS];
          break;

        case FLIGHTCONTROL_MODE_ATTITUDE:
          flight_control_command.command = FLIGHTCONTROL_COMMAND_ATTITUDE;
          flight_control_command.attitude.pitch = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_ELEVATOR] * PITCH_ATTITUDE_STICK_SCALER;
          flight_control_command.attitude.roll = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_AILERON] * ROLL_ATTITUDE_STICK_SCALER;;
          flight_control_command.attitude.yaw = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_RUDDER];
          flight_control_command.attitude.throttle = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_THROTTLE];
          flight_control_command.attitude.flap = crsf_status.channel_data.channels[RC_INPUT_CHANNEL_FLAPS];
          break;
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
        control_outputs.throttle = flight_control_command.direct.throttle;
        control_outputs.flap = flight_control_command.direct.flap * FLAP_SCALER;
        last_flight_control_command = FLIGHTCONTROL_COMMAND_DIRECT;
        break;

      // Rate based (degrees per second)
      case FLIGHTCONTROL_COMMAND_RATE:
        control_outputs.yaw = flight_control_command.rate.yaw_rate;
        control_outputs.pitch = Pid_Calculate(&pitch_rate_pid, flight_control_command.rate.pitch_rate, imu_ahrs_status.pitch_rate);
        control_outputs.roll = Pid_Calculate(&roll_rate_pid, flight_control_command.rate.roll_rate, imu_ahrs_status.roll_rate);
        control_outputs.throttle = flight_control_command.rate.throttle;
        control_outputs.flap = flight_control_command.rate.flap * FLAP_SCALER;
        last_flight_control_command = FLIGHTCONTROL_COMMAND_RATE;
        break;

      case FLIGHTCONTROL_COMMAND_ATTITUDE:
        ComputeAttitude(&control_outputs, &imu_ahrs_status, &flight_control_command.attitude);
        control_outputs.flap = 0;
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

  actuators[SERVO_ACTUATOR_OUTPUT_THROTTLE] = controls->throttle;
  actuators[SERVO_ACTUATOR_OUTPUT_ELEVATOR] = controls->pitch;
  actuators[SERVO_ACTUATOR_OUTPUT_RUDDER] = controls->yaw;
  actuators[SERVO_ACTUATOR_OUTPUT_AILERON_LEFT] = controls->roll + controls->flap;
  actuators[SERVO_ACTUATOR_OUTPUT_AILERON_RIGHT] = -controls->roll + controls->flap;

  ServoActuatorTranslate((float*)&actuators, (ServoActuatorTranslation_t*)&servo_accuator_translations, SERVO_ACTUATOR_COUNT);
}
