#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "i2cdevlib/MPU6050/MPU6050.h"
#include "Log.h"
#include "Madgwick/Fusion.h"

static FusionBias fusion_bias;
static FusionAhrs fusion_ahrs;

#define STATIONARY_THRESHOLD .5f

#define MPU6050_DPS_PER_LSB 0.06103515625f
#define MPU6050_GS_PER_LSB  0.00048828125f

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

// Align mpu6050, 180
static const FusionRotationMatrix mpu6050_alignment =
{
  .array =
  {
    0.0f, -1.0f, 0.0f, 
    1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f
  } 
};

// Mag hardiron in uT
static const FusionVector3 hardIronBias =
{
  .axis.x = 0.0f,
  .axis.y = 0.0f,
  .axis.z = 0.0f,
};

static float madgwick_sample_period_s;

enum IMUAHRS_FLAGS
{
  IMUAHRS_FLAGS_MPU6050_FAIL = (1 << 0),
};

static uint32_t flags = 0;

static const char* TAG = "FLIGHTCONTROL";

#define IMUAHRS_PERIOD   (5 / portTICK_PERIOD_MS)

static void ImuAhrsTask(void* arg);

void ImuAhrs_Init(void)
{
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
  
  while (true)
  {
    // Get accel/gyro
    MPU6050_getMotion6(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);
    
    // Load accel data
    uncalibrated_accel.axis.x = accel[0];
    uncalibrated_accel.axis.y = accel[1];
    uncalibrated_accel.axis.z = accel[2];
    
    // Load gyro data
    uncalibrated_gyro.axis.x = gyro[0];
    uncalibrated_gyro.axis.y = gyro[1];
    uncalibrated_gyro.axis.z = gyro[2] + 150;
    
    calibrated_gyro = FusionCalibrationInertial(uncalibrated_gyro, mpu6050_alignment, gyroscope_sensitivity, FUSION_VECTOR3_ZERO);
    calibrated_accel = FusionCalibrationInertial(uncalibrated_accel, mpu6050_alignment, accelerometer_sensitivity, FUSION_VECTOR3_ZERO);
    //calibrated_mag = FusionCalibrationMagnetic(uncalibrated_mag, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

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

    vTaskDelay(IMUAHRS_PERIOD);
  }
}