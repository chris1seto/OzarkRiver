#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Log.h"
#include "Madgwick/Fusion.h"
#include "Ticks.h"
#include "I2c1.h"
#include "ImuAhrs.h"
#include "Bits.h"
#include "Mpu6000.h"
#include "Spi1.h"

static Mpu6000Instance_t mpu6000_i = {0};

static QueueHandle_t imu_ahrs_status_queue;
static ImuAhrsStatus_t imu_ahrs_status = {0};

#define MPU6050_DPS_PER_LSB 0.06103515625f
#define MPU6050_GS_PER_LSB  0.00048828125

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
static const FusionRotationMatrix accel_gyro_alignment =
{
  .array =
  {
    0.0f, -1.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f
  }
};

// Align hmc5983 90
static const FusionRotationMatrix mag_alignment =
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

static uint32_t flags = 0;

static const char* TAG = "IMUAHRS";

#define IMUAHRS_PERIOD   (5 / portTICK_PERIOD_MS)

static void ImuAhrsTask(void* arg);

#define INIT_COUNT_LIMIT 5

static bool Mpu6000_XAction_Shim(const uint8_t* write, uint8_t* const read, const uint32_t size);
static void Mpu6000_Delay_Shim(const uint32_t us);

void ImuAhrs_Init(void)
{
  imu_ahrs_status_queue = xQueueCreate(1, sizeof(ImuAhrsStatus_t));

  // Init Madgwick
  // Initialise gyroscope bias correction
  madgwick_sample_period_s = IMUAHRS_PERIOD * .001;
  FusionBiasInitialise(&fusion_bias, STATIONARY_THRESHOLD, madgwick_sample_period_s);

  // Initialise AHRS
  FusionAhrsInitialise(&fusion_ahrs, 0.5f);

  // Set optional magnetic field limits
  //FusionAhrsSetMagneticField(&fusion_ahrs, 20.0f, 70.0f);

  mpu6000_i.delay = Mpu6000_Delay_Shim;
  mpu6000_i.xaction = Mpu6000_XAction_Shim;
  Mpu6000_Init(&mpu6000_i);

  xTaskCreate(ImuAhrsTask, TAG, 700, NULL, 0, NULL);
}

static bool Mpu6000_XAction_Shim(const uint8_t* write, uint8_t* const read, const uint32_t size)
{
  return Spi1_Transaction(SPI1_TARGET_MPU6000, write, read, size);
}

static void Mpu6000_Delay_Shim(const uint32_t us)
{
  HAL_Delay(us);
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

  Mpu6000AccelXyz_t accel;
  Mpu6000GyroXyz_t gyro;
  int16_t accel_gyro_temp;

  while (true)
  {
    if (!Mpu6000_ReadAccelTempGyro(&mpu6000_i, &accel, &accel_gyro_temp, &gyro))
    {
      Bits_Set(&flags, IMUAHRS_FLAGS_ACCEL_FAIL | IMUAHRS_FLAGS_GYRO_FAIL);
    }
    else
    {
      Bits_Clear(&flags, IMUAHRS_FLAGS_ACCEL_FAIL | IMUAHRS_FLAGS_GYRO_FAIL);
    }

    raw_imu_data.accel.s.x = accel.x;
    raw_imu_data.accel.s.y = accel.y;
    raw_imu_data.accel.s.z = accel.z;

    raw_imu_data.gyro.s.x = gyro.x;
    raw_imu_data.gyro.s.y = gyro.y;
    raw_imu_data.gyro.s.z = gyro.z;

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
    uncalibrated_mag.axis.x = 0;
    uncalibrated_mag.axis.y = 0;
    uncalibrated_mag.axis.z = 0;

    calibrated_gyro = FusionCalibrationInertial(uncalibrated_gyro, accel_gyro_alignment, gyroscope_sensitivity, FUSION_VECTOR3_ZERO);
    calibrated_accel = FusionCalibrationInertial(uncalibrated_accel, accel_gyro_alignment, accelerometer_sensitivity, FUSION_VECTOR3_ZERO);
    //calibrated_mag = FusionCalibrationMagnetic(uncalibrated_mag, mag_alignment, hard_iron_bias);

    // Update gyroscope bias correction
    calibrated_gyro = FusionBiasUpdate(&fusion_bias, calibrated_gyro);

    // Update AHRS
    FusionAhrsUpdateWithoutMagnetometer(&fusion_ahrs, calibrated_gyro, calibrated_accel, madgwick_sample_period_s);

    // Generate euler angles
    euler_angles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusion_ahrs));

    // Load status
    imu_ahrs_status.timestamp = Ticks_Now();
    imu_ahrs_status.flags = flags;
    imu_ahrs_status.pitch = euler_angles.angle.pitch;
    imu_ahrs_status.roll = euler_angles.angle.roll;
    imu_ahrs_status.yaw = euler_angles.angle.yaw;
    imu_ahrs_status.pitch_rate = calibrated_gyro.axis.y;
    imu_ahrs_status.roll_rate = calibrated_gyro.axis.x;
    imu_ahrs_status.yaw_rate = calibrated_gyro.axis.z;

    xQueueOverwrite(imu_ahrs_status_queue, &imu_ahrs_status);

    vTaskDelay(IMUAHRS_PERIOD);
  }
}

bool ImuAhrs_GetStatus(ImuAhrsStatus_t* const status)
{
  return (xQueueReceive(imu_ahrs_status_queue, status, 0) == pdTRUE);
}
