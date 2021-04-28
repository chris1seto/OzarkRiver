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

static QueueHandle_t imu_ahrs_status_queue;
static ImuAhrsStatus_t imu_ahrs_status = {0};

static FusionBias fusion_bias;
static FusionAhrs fusion_ahrs;

#define STATIONARY_THRESHOLD .5f

#define MPU6050_DPS_PER_LSB 0.06103515625f
#define MPU6050_GS_PER_LSB  0.00048828125f
#define HMC5983_

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

// Align hmc5983
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

enum IMUAHRS_FLAGS
{
  IMUAHRS_FLAGS_MPU6050_FAIL = (1 << 0),
};

static uint32_t flags = 0;

static const char* TAG = "IMUAHRS";

#define IMUAHRS_PERIOD   (5 / portTICK_PERIOD_MS)

static void ImuAhrsTask(void* arg);

void ImuAhrs_Init(void)
{
  imu_ahrs_status_queue = xQueueCreate(1, sizeof(ImuAhrsStatus_t));

  // Init MPU6050
  MPU6050_initialize(MPU6050_DEFAULT_ADDRESS);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
	MPU6050_setSleepEnabled(false);
	MPU6050_setDLPFMode(0);

  MPU6050_setXGyroOffset(0);
  MPU6050_setYGyroOffset(0);
  MPU6050_setZGyroOffset(0);

  if (MPU6050_testConnection())
  {
    LOG_I(TAG, "MPU6050 OK");
  }
  else
  {
    LOG_E(TAG, "MPU6050 fail");
  }

  // Init Hmc5983
  hmc5983_inst.address = HMC5983_ADDRESS;
  hmc5983_inst.read = I2c1_ReadMemory8;
  hmc5983_inst.write = I2c1_WriteMemory8;

  // Init Madgwick
  // Initialise gyroscope bias correction
  madgwick_sample_period_s = IMUAHRS_PERIOD * .001;
  FusionBiasInitialise(&fusion_bias, STATIONARY_THRESHOLD, madgwick_sample_period_s);

  // Initialise AHRS
  FusionAhrsInitialise(&fusion_ahrs, 0.5f);

  // Set optional magnetic field limits
  //FusionAhrsSetMagneticField(&fusion_ahrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT

  xTaskCreate(ImuAhrsTask, TAG, 512, NULL, 0, NULL);
}

static void ImuAhrsTask(void* arg)
{
  int16_t accel[3];
  int16_t gyro[3];
  FusionVector3 uncalibrated_gyro;
  FusionVector3 uncalibrated_accel;
  FusionVector3 uncalibrated_mag;
  FusionEulerAngles euler_angles;
  FusionVector3 calibrated_gyro;
  FusionVector3 calibrated_accel;
  //FusionVector3 calibrated_mag;
  bool x = false;
  bool is_converged = false;
  uint32_t i;
  bool axes_in_range;

  while (true)
  {
    // Get accel/gyro
    MPU6050_getMotion6(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);

    // Shim measurements
    for (i = 0; i < 3; i++)
    {
      accel[i] -= accel_offset[i];
      gyro[i] -= gyro_offset[i];
    }

    // Simple mechanism to calibrate gyro drift offset
    if (!is_calibrated)
    {
      axes_in_range = true;

      for (i = 0; i < 3; i++)
      {
        if (gyro[i] > 0)
        {
          gyro_offset[i]++;
        }
        else
        {
          gyro_offset[i]--;
        }

        if (gyro[i] > GYRO_DEADZONE || gyro[i] < -GYRO_DEADZONE)
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
        }
      }
      else
      {
        is_converged = false;
      }

      continue;
    }

    // Load accel data
    uncalibrated_accel.axis.x = accel[0];
    uncalibrated_accel.axis.y = accel[1];
    uncalibrated_accel.axis.z = accel[2];

    // Load gyro data
    uncalibrated_gyro.axis.x = gyro[0];
    uncalibrated_gyro.axis.y = gyro[1];
    uncalibrated_gyro.axis.z = gyro[2];

    calibrated_gyro = FusionCalibrationInertial(uncalibrated_gyro, mpu6050_alignment, gyroscope_sensitivity, FUSION_VECTOR3_ZERO);
    calibrated_accel = FusionCalibrationInertial(uncalibrated_accel, mpu6050_alignment, accelerometer_sensitivity, FUSION_VECTOR3_ZERO);
    //calibrated_mag = FusionCalibrationMagnetic(uncalibrated_mag, FUSION_ROTATION_MATRIX_IDENTITY, hard_iron_bias);

    // Update gyroscope bias correction
    calibrated_gyro = FusionBiasUpdate(&fusion_bias, calibrated_gyro);

    // Update AHRS
    //FusionAhrsUpdate(&fusion_ahrs, calibrated_gyro, calibrated_accel, calibrated_mag, madgwick_sample_period_s);
    FusionAhrsUpdateWithoutMagnetometer(&fusion_ahrs, calibrated_gyro, calibrated_accel, madgwick_sample_period_s);

    // Print Euler angles
    euler_angles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusion_ahrs));

    if (x)
    //printf("%.2f %.2f %.2f\r\n", accel[0] * MPU6050_GS_PER_LSB, accel[1] *MPU6050_GS_PER_LSB, accel[2]*MPU6050_GS_PER_LSB);
    //printf("%04.2f %04.2f %04.2f\r\n", gyro[0] * MPU6050_DPS_PER_LSB, gyro[1] * MPU6050_DPS_PER_LSB, gyro[2] * MPU6050_DPS_PER_LSB);
    printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", euler_angles.angle.roll, euler_angles.angle.pitch, euler_angles.angle.yaw);

    x = !x;

    imu_ahrs_status.timestamp = Ticks_Now();
    imu_ahrs_status.pitch = euler_angles.angle.pitch;
    imu_ahrs_status.roll = euler_angles.angle.roll;
    imu_ahrs_status.yaw = euler_angles.angle.yaw;
    imu_ahrs_status.pitch_rate = 0;
    imu_ahrs_status.roll_rate = 0;
    imu_ahrs_status.yaw_rate = 0;

    vTaskDelay(IMUAHRS_PERIOD);
  }
}

bool ImuAhrs_GetStatus(ImuAhrsStatus_t* const status)
{
  return (xQueueReceive(imu_ahrs_status_queue, status, 0) == pdTRUE);
}
