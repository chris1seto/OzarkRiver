#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "i2cdevlib/MPU6050/MPU6050.h"
#include "Hmc5983.h"
#include "Log.h"
#include "Madgwick/Fusion.h"
#include "Ticks.h"
#include "I2c1.h"
#include "ImuAhrs.h"
#include "Bits.h"

static QueueHandle_t imu_ahrs_status_queue;
static ImuAhrsStatus_t imu_ahrs_status = {0};

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} Xyz16_t;

typedef union
{
  Xyz16_t s;
  int16_t a[3];
} Xyz16u_t;

typedef struct
{
  Xyz16u_t accel;
  Xyz16u_t gyro;
  Xyz16u_t mag;
} Imu_t;

static FusionBias fusion_bias;
static FusionAhrs fusion_ahrs;

#define STATIONARY_THRESHOLD .5f

#define MPU6050_DPS_PER_LSB 0.06103515625f
#define MPU6050_GS_PER_LSB  0.00048828125
#define HMC_TICKS_TO_uT(x)  (x * (1.0f / 1090.0f) * 100.0f)

static int16_t gyro_offset[3] = {0, 0, 0};
static int16_t accel_offset[3] = {0, 0, 0};

#define CONVERGED_TIMEOUT (5000 / portTICK_PERIOD_MS)
static TickType_t converged_start_time = 0;

#define GYRO_DEADZONE 10

static bool is_calibrated = false;

// Degrees per second per LSB
static const FusionVector3 gyroscope_sensitivity =
{
  .axis.x = MPU6050_DPS_PER_LSB,
  .axis.y = MPU6050_DPS_PER_LSB,
  .axis.z = MPU6050_DPS_PER_LSB,
};

// G's per LSB
static const FusionVector3 accelerometer_sensitivity =
{
  .axis.x = MPU6050_GS_PER_LSB,
  .axis.y = MPU6050_GS_PER_LSB,
  .axis.z = MPU6050_GS_PER_LSB,
};

// Align mpu6050, 90
static const FusionRotationMatrix mpu6050_alignment =
{
  .array =
  {
    0.0f, -1.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f
  }
};

// Align hmc5983 90
static const FusionRotationMatrix hmc5983_alignment =
{
  .array =
  {
    0.0f, -1.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f
  }
};

// Mag hardiron in uT
static const FusionVector3 hard_iron_bias =
{
  .axis.x = 0.0f,
  .axis.y = 0.0f,
  .axis.z = 0.0f,
};

static float madgwick_sample_period_s;

static Hmc5983Instance_t hmc5983_inst = {0};

static uint32_t flags = 0;

static const char* TAG = "IMUAHRS";

#define IMUAHRS_PERIOD   (5 / portTICK_PERIOD_MS)

static void ImuAhrsTask(void* arg);

#define INIT_COUNT_LIMIT 5

static bool Mpu6050Init()
{
  MPU6050_initialize(MPU6050_DEFAULT_ADDRESS);

  if (MPU6050_testConnection())
  {
    LOG_I(TAG, "MPU6050 OK");
  }
  else
  {
    LOG_E(TAG, "MPU6050 fail");
    return false;
  }

  MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  HAL_Delay(5);
  MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  HAL_Delay(5);
  MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  HAL_Delay(5);
  MPU6050_setSleepEnabled(false);
  HAL_Delay(5);
  MPU6050_setDLPFMode(3);
  HAL_Delay(5);

  MPU6050_setXGyroOffset(0);
  MPU6050_setYGyroOffset(0);
  MPU6050_setZGyroOffset(0);

  MPU6050_setSleepEnabled(false);

  return true;
}

static bool Hmc5983Init(void)
{
  if (Hmc5983_CheckIdentification(&hmc5983_inst))
  {
    LOG_I(TAG, "HMC5983 OK");
  }
  else
  {
    LOG_E(TAG, "HMC5983 fail");
    return false;
  }

  if (!Hmc5983_SetConfigA(&hmc5983_inst, HMC5983_MEASUREMENT_MODE_NORMAL | HMC5983_DATA_OUTPUT_RATE_220_0 | HMC5983_TEMPERATURE_ENABLE_ENABLE | HMC5983_SAMPLES_AVERAGED_1))
  {
    LOG_E(TAG, "Hmc5983_SetConfigA failed");
    return false;
  }

  if (!Hmc5983_SetConfigB(&hmc5983_inst, HMC5983_GAIN_1))
  {
    LOG_E(TAG, "Hmc5983_SetConfigB failed");
    return false;
  }

  if (!Hmc5983_SetMode(&hmc5983_inst, HMC5983_OPERATING_MODE_CONTINUOUS))
  {
    LOG_E(TAG, "Hmc5983_SetMode failed");
    return false;
  }

  return true;
}

void ImuAhrs_Init(void)
{
  uint32_t i;

  imu_ahrs_status_queue = xQueueCreate(1, sizeof(ImuAhrsStatus_t));

  // Init MPU6050
  for (i = 0; i < INIT_COUNT_LIMIT; i++)
  {
    if (Mpu6050Init())
    {
      // This is a hack, but for some reason the mpu6050 doesn't return good data without this routine running twice.
      // It's probably a timing thing or who knows what, I haven't dug into it at all and I don't really care to
      // the Mpu6050 is EOL and I'm only using it because it happens to be on this board, so I'm not going to put effort into fixing this
      // As long as it continues to work after being init'd twice  ¯\_(ツ)_/¯
      Mpu6050Init();

      break;
    }

    // If we're here, init failed
    HAL_Delay(100);
  }

  if (i == INIT_COUNT_LIMIT)
  {
    Bits_Set(&flags, IMUAHRS_FLAGS_MPU6050_FAIL);
  }

  // Init Hmc5983
  hmc5983_inst.address = HMC5983_ADDRESS;
  hmc5983_inst.read = I2c1_ReadMemory8;
  hmc5983_inst.write = I2c1_WriteMemory8;

  for (i = 0; i < INIT_COUNT_LIMIT; i++)
  {
    if (Hmc5983Init())
    {
      break;
    }

    // If we're here, init failed
    HAL_Delay(100);
  }

  if (i == INIT_COUNT_LIMIT)
  {
    Bits_Set(&flags, IMUAHRS_FLAGS_HMC5983_FAIL);
  }

  // Init Madgwick
  // Initialise gyroscope bias correction
  madgwick_sample_period_s = IMUAHRS_PERIOD * .001;
  FusionBiasInitialise(&fusion_bias, STATIONARY_THRESHOLD, madgwick_sample_period_s);

  // Initialise AHRS
  FusionAhrsInitialise(&fusion_ahrs, 0.5f);

  // Set optional magnetic field limits
  FusionAhrsSetMagneticField(&fusion_ahrs, 20.0f, 70.0f);

  xTaskCreate(ImuAhrsTask, TAG, 700, NULL, 0, NULL);
}

static bool is_converged = false;



static void Calibrate(const Imu_t* raw_imu_data)
{
  uint32_t i;
  bool axes_in_range;

  // Simple mechanism to calibrate gyro drift offset
  if (!is_calibrated)
  {
    Bits_Set(&flags, IMUAHRS_FLAGS_CALIBRATING);
    axes_in_range = true;

    for (i = 0; i < 3; i++)
    {
      if (raw_imu_data->gyro.a[i] > 0)
      {
        gyro_offset[i]++;
      }
      else
      {
        gyro_offset[i]--;
      }

      if (raw_imu_data->gyro.a[i] > GYRO_DEADZONE || raw_imu_data->gyro.a[i] < -GYRO_DEADZONE)
      {
        axes_in_range = false;
      }
    }

    if (axes_in_range)
    {
      if (!is_converged)
      {
        Ticks_Reset(&converged_start_time);
      }

      is_converged = true;

      if (Ticks_IsExpired(converged_start_time, CONVERGED_TIMEOUT))
      {
        LOG_W(TAG, "Gyro calibrated! (%i, %i, %i)", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
        is_calibrated = true;
        Bits_Clear(&flags, IMUAHRS_FLAGS_CALIBRATING);
      }
    }
    else
    {
      is_converged = false;
    }
  }
}

static void ImuAhrsTask(void* arg)
{
  uint32_t i;

  Imu_t raw_imu_data;

  FusionVector3 uncalibrated_gyro;
  FusionVector3 uncalibrated_accel;
  FusionVector3 uncalibrated_mag;
  FusionEulerAngles euler_angles;
  FusionVector3 calibrated_gyro;
  FusionVector3 calibrated_accel;
  FusionVector3 calibrated_mag;

  while (true)
  {
    // Get accel/gyro
    MPU6050_getMotion6(&raw_imu_data.accel.s.x, &raw_imu_data.accel.s.y, &raw_imu_data.accel.s.z,
      &raw_imu_data.gyro.s.x, &raw_imu_data.gyro.s.y, &raw_imu_data.gyro.s.z);

    // Get mag
    Hmc5983_GetMag(&hmc5983_inst, &raw_imu_data.mag.s.x, &raw_imu_data.mag.s.y, &raw_imu_data.mag.s.z);

    // Shim measurements
    for (i = 0; i < 3; i++)
    {
      raw_imu_data.accel.a[i] -= accel_offset[i];
      raw_imu_data.gyro.a[i] -= gyro_offset[i];
    }

    if (!is_calibrated)
    {
      Calibrate(&raw_imu_data);

      // Load status
      imu_ahrs_status.timestamp = Ticks_Now();
      imu_ahrs_status.flags = flags;
      imu_ahrs_status.pitch = 0;
      imu_ahrs_status.roll = 0;
      imu_ahrs_status.yaw = 0;
      imu_ahrs_status.pitch_rate = 0;
      imu_ahrs_status.roll_rate = 0;
      imu_ahrs_status.yaw_rate = 0;

      xQueueOverwrite(imu_ahrs_status_queue, &imu_ahrs_status);

      vTaskDelay(IMUAHRS_PERIOD);

      continue;
    }

    // Load accel data
    uncalibrated_accel.axis.x = raw_imu_data.accel.s.x;
    uncalibrated_accel.axis.y = raw_imu_data.accel.s.y;
    uncalibrated_accel.axis.z = raw_imu_data.accel.s.z;

    // Load gyro data
    uncalibrated_gyro.axis.x = raw_imu_data.gyro.s.x;
    uncalibrated_gyro.axis.y = raw_imu_data.gyro.s.y;
    uncalibrated_gyro.axis.z = raw_imu_data.gyro.s.z;

    // Load mag data
    uncalibrated_mag.axis.x = HMC_TICKS_TO_uT(raw_imu_data.mag.s.x);
    uncalibrated_mag.axis.y = HMC_TICKS_TO_uT(raw_imu_data.mag.s.y);
    uncalibrated_mag.axis.z = HMC_TICKS_TO_uT(raw_imu_data.mag.s.z);

    calibrated_gyro = FusionCalibrationInertial(uncalibrated_gyro, mpu6050_alignment, gyroscope_sensitivity, FUSION_VECTOR3_ZERO);
    calibrated_accel = FusionCalibrationInertial(uncalibrated_accel, mpu6050_alignment, accelerometer_sensitivity, FUSION_VECTOR3_ZERO);
    calibrated_mag = FusionCalibrationMagnetic(uncalibrated_mag, hmc5983_alignment, hard_iron_bias);

    // Update gyroscope bias correction
    calibrated_gyro = FusionBiasUpdate(&fusion_bias, calibrated_gyro);

    // Update AHRS
    FusionAhrsUpdate(&fusion_ahrs, calibrated_gyro, calibrated_accel, calibrated_mag, madgwick_sample_period_s);

    // Generate euler angles
    euler_angles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusion_ahrs));

    // Load status
    imu_ahrs_status.timestamp = Ticks_Now();
    imu_ahrs_status.flags = flags;
    imu_ahrs_status.pitch = -euler_angles.angle.pitch;
    imu_ahrs_status.roll = euler_angles.angle.roll;
    imu_ahrs_status.yaw = -euler_angles.angle.yaw;
    imu_ahrs_status.pitch_rate = calibrated_gyro.axis.y;
    imu_ahrs_status.roll_rate = calibrated_gyro.axis.x;
    imu_ahrs_status.yaw_rate = -calibrated_gyro.axis.z;

    xQueueOverwrite(imu_ahrs_status_queue, &imu_ahrs_status);

    vTaskDelay(IMUAHRS_PERIOD);
  }
}

bool ImuAhrs_GetStatus(ImuAhrsStatus_t* const status)
{
  return (xQueueReceive(imu_ahrs_status_queue, status, 0) == pdTRUE);
}
