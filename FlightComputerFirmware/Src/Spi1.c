#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Ticks.h"
#include "Spi1.h"

static SPI_HandleTypeDef spi_handle;

void Spi1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // Enable clocks
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Init SPI1 GPIO
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;

  // SPI1_MISO
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // SPI1_MOSI
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Alternate =  GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // SPI1_SCK
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Alternate =  GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Target MPU6000
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Alternate =  0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, true);

  // Init SPI
  spi_handle.Instance               = SPI1;
  spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  spi_handle.Init.Direction         = SPI_DIRECTION_2LINES;
  spi_handle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  spi_handle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  spi_handle.Init.DataSize          = SPI_DATASIZE_8BIT;
  spi_handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  spi_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
  spi_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  spi_handle.Init.CRCPolynomial     = 7;
  spi_handle.Init.NSS               = SPI_NSS_SOFT;
  spi_handle.Init.Mode              = SPI_MODE_MASTER;
  HAL_SPI_Init(&spi_handle);
}

static void SelectTarget(const enum SPI1_TARGET target)
{
  switch (target)
  {
    case SPI1_TARGET_MPU6000:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, false);
      break;
  }
}

static void DeSelectTarget(const enum SPI1_TARGET target)
{
  switch (target)
  {
    case SPI1_TARGET_MPU6000:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, true);
      break;
  }
}

bool Spi1_Transaction(const enum SPI1_TARGET target, const uint8_t* tx, uint8_t* const rx, const uint32_t size)
{
  bool result;

  SelectTarget(target);

  result = (HAL_SPI_TransmitReceive(&spi_handle, (uint8_t*)tx, (uint8_t*)rx, size, 5000) == HAL_OK);

  DeSelectTarget(target);

  return result;
}
