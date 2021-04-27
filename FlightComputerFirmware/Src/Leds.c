#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f3xx_hal.h>
#include "Leds.h"

void Leds_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Init LED pins
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  // PB3 RED
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  Leds_Off(LED_RED);
}

void Leds_On(const uint32_t leds)
{
  // PB3 0
  if (leds & LED_RED)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, false);
  }
}

void Leds_Off(const uint32_t leds)
{
  // PB3 0
  if (leds & LED_RED)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, true);
  }
}

void Leds_Toggle(const uint32_t leds, const bool enabled)
{
  // PB3 0
  if (leds & LED_RED)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, !enabled);
  }
}
