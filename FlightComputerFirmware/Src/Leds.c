#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include "Leds.h"

void Leds_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Init LED pins
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  // PA14 Blue
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // PA13 Green
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  Leds_Off(LED_BLUE);
  Leds_Off(LED_GREEN);
}

void Leds_On(const uint32_t leds)
{
  if (leds & LED_BLUE)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, false);
  }

  if (leds & LED_GREEN)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, false);
  }
}

void Leds_Off(const uint32_t leds)
{

  if (leds & LED_BLUE)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, true);
  }
  
  if (leds & LED_GREN)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, true);
  }
}

void Leds_Toggle(const uint32_t leds, const bool enabled)
{
  if (leds & LED_BLUE)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, !enabled);
  }

  if (leds & LED_GREEN)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, !enabled);
  }
}
