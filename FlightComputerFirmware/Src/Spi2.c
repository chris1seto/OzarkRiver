#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Ticks.h"
#include "Spi2.h"

static SPI_HandleTypeDef spi_handle;

void Spi2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // Enable clocks
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Init SPI2 GPIO
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;

  // SPI2_MISO
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // SPI2_MOSI
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Alternate =  GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // SPI2_SCK
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Alternate =  GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Target Winbond CS PB12
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Alternate =  0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, true);

  // Init SPI
  spi_handle.Instance               = SPI2;
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

static void SelectTarget(const enum SPI2_TARGET target)
{
  switch (target)
  {
    case SPI2_TARGET_WINBOND:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, false);
      break;
  }
}

static void DeSelectTarget(const enum SPI2_TARGET target)
{
  switch (target)
  {
    case SPI2_TARGET_WINBOND:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, true);
      break;
  }
}

bool Spi2_Transaction(const enum SPI2_TARGET target, const uint8_t* tx, uint8_t* const rx, const uint32_t size)
{
  bool result;

  SelectTarget(target);

  result = (HAL_SPI_TransmitReceive(&spi_handle, (uint8_t*)tx, (uint8_t*)rx, size, 5000) == HAL_OK);

  DeSelectTarget(target);

  return result;
}
