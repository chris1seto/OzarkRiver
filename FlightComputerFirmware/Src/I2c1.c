#include <stdio.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include "I2c1.h"

static I2C_HandleTypeDef  i2c1_handle;

void I2c1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  // Common GPIO settings
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  // SDA
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // SCL
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Init I2C
  i2c1_handle.Instance = I2C1;
  i2c1_handle.Init.ClockSpeed = 400000;
  i2c1_handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  i2c1_handle.Init.OwnAddress1 = 0;
  i2c1_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  i2c1_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  i2c1_handle.Init.OwnAddress2 = 0;
  i2c1_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  i2c1_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&i2c1_handle);

}

bool I2c1_WriteMemory8(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t* data, const uint32_t size)
{
  if (HAL_I2C_Mem_Write(&i2c1_handle, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, size, 10000) != HAL_OK)
  {
    return false;
  }

  return true;
}

bool I2c1_ReadMemory8(const uint8_t dev_addr, const uint8_t reg_addr, uint8_t* const data, const uint32_t size)
{
  if (HAL_I2C_Mem_Read(&i2c1_handle, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, size, 10000) != HAL_OK)
  {
    return false;
  }

  return true;
}

bool I2c1_Write(const uint8_t dev_addr, const uint8_t* data, const uint32_t size)
{
  if (HAL_I2C_Master_Transmit(&i2c1_handle, dev_addr, (uint8_t*)data, size, 10000) != HAL_OK)
  {
    return false;
  }

  return true;
}

bool I2c1_Read(const uint8_t dev_addr, uint8_t* const data, const uint32_t size)
{
  if (HAL_I2C_Master_Receive(&i2c1_handle, dev_addr, (uint8_t*)data, size, 10000) != HAL_OK)
  {
    return false;
  }

  return true;
}
