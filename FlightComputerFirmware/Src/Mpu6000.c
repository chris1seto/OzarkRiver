#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "Mpu6000.h"

#define MPU_RA_WHO_AM_I         0x75
#define MPU_RA_WHO_AM_I_LEGACY  0x00

#define MPUx0x0_WHO_AM_I_CONST              (0x68)
// RA = Register Address

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    0x04
#define BIT_ACC                     0x02
#define BIT_TEMP                    0x01

enum gyro_fsr_e
{
  INV_FSR_250DPS = 0,
  INV_FSR_500DPS,
  INV_FSR_1000DPS,
  INV_FSR_2000DPS,
  NUM_GYRO_FSR
};

enum fchoice_b
{
  FCB_DISABLED = 0,
  FCB_8800_32,
  FCB_3600_32
};

enum clock_sel_e
{
  INV_CLK_INTERNAL = 0,
  INV_CLK_PLL,
  NUM_CLK
};

enum accel_fsr_e
{
  INV_FSR_2G = 0,
  INV_FSR_4G,
  INV_FSR_8G,
  INV_FSR_16G,
  NUM_ACCEL_FSR
};

#define X_BUFFER_SIZE 128
static uint8_t x_buffer_rx[X_BUFFER_SIZE];
static uint8_t x_buffer_tx[X_BUFFER_SIZE];

#define ADDRESS_WORD_SIZE 1

static bool XAction(const Mpu6000Instance_t* i, const uint8_t* write, uint8_t* const read, const uint32_t size);
static void Delay(const Mpu6000Instance_t* i, const uint32_t us);

static bool XAction(const Mpu6000Instance_t* i, const uint8_t* write, uint8_t* const read, const uint32_t size)
{
  if (i->xaction == NULL)
  {
    return false;
  }

  return i->xaction(write, read, size);
}

static void Delay(const Mpu6000Instance_t* i, const uint32_t us)
{
  if (i->delay == NULL)
  {
    return;
  }

  i->delay(us);
}

static bool Read(const Mpu6000Instance_t* i,const uint8_t reg, uint8_t* const data, const uint32_t size)
{
  x_buffer_tx[0] = (1 << 7) | (reg & 0x7f);
  if (!XAction(i, x_buffer_tx, x_buffer_rx, size + ADDRESS_WORD_SIZE))
  {
    return false;
  }

  memcpy(data, x_buffer_rx + ADDRESS_WORD_SIZE, size);
  return true;
}

static bool Write(const Mpu6000Instance_t* i, const uint8_t reg, const uint8_t* data, const uint32_t size)
{
  x_buffer_tx[0] = (0 << 7) | (reg & 0x7f);
  memcpy(x_buffer_tx + ADDRESS_WORD_SIZE, data, size);

  return XAction(i, x_buffer_tx, x_buffer_rx, size + ADDRESS_WORD_SIZE);
}

static bool ReadS(const Mpu6000Instance_t* i, const uint8_t reg, uint8_t* const data)
{
  return Read(i, reg, data, 1);
}

static bool WriteS(const Mpu6000Instance_t* i, const uint8_t reg, const uint8_t data)
{
  return Write(i, reg, &data, 1);
}

bool Mpu6000_Init(const Mpu6000Instance_t* i)
{
  // Reset configuration
  WriteS(i, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
  Delay(i, 100);

  // Reset signal paths
  WriteS(i, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
  Delay(i, 100);

  // Clock Source PPL with Z axis gyro reference
  WriteS(i, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);

  // Disable Primary I2C Interface
  WriteS(i, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);

  WriteS(i, MPU_RA_PWR_MGMT_2, 0x00);

  // Accel Sample Rate 1kHz
  // Gyroscope Output Rate =  1kHz when the DLPF is enabled
  WriteS(i, MPU_RA_SMPLRT_DIV, 0);

  // Gyro +/- 2000 DPS Full Scale
  WriteS(i, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);

  // Accel +/- 16 G Full Scale
  WriteS(i, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);

  return true;
}

#define ACCEL_GYRO_WORD_SIZE 2
bool Mpu6000_ReadAccelGyro(const Mpu6000Instance_t* i, Mpu6000AccelXyz_t* const accel, Mpu6000GyroXyz_t* const gyro)
{
  uint8_t accel_gyro_raw[6 * ACCEL_GYRO_WORD_SIZE];
  
  if (!Read(i, MPU_RA_ACCEL_XOUT_H, (uint8_t*)&accel_gyro_raw, 6 * ACCEL_GYRO_WORD_SIZE))
  {
    return false;
  }
  
  accel->x = (accel_gyro_raw[0] << 8) | accel_gyro_raw[1];
  accel->y = (accel_gyro_raw[2] << 8) | accel_gyro_raw[3];
  accel->z = (accel_gyro_raw[4] << 8) | accel_gyro_raw[5];
  
  gyro->x = (accel_gyro_raw[6] << 8) | accel_gyro_raw[7];
  gyro->y = (accel_gyro_raw[8] << 8) | accel_gyro_raw[9];
  gyro->z = (accel_gyro_raw[10] << 8) | accel_gyro_raw[11];
  
  return true;
}