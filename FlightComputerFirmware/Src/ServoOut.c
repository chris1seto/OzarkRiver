#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include "ServoOut.h"

static TIM_HandleTypeDef tim4_handle;
static TIM_HandleTypeDef tim3_handle;
static TIM_HandleTypeDef tim8_handle;
static TIM_HandleTypeDef tim12_handle;

#define SERVO_PERIOD_MS   20

// 84Mhz
#define TIM_4_PRESCALER   13
#define TIM_4_PERIOD      64536
#define TIM_4_MS_TO_TICKS(x) (uint16_t)(x * 6453.6)

// 84Mhz
#define TIM_3_PRESCALER   13
#define TIM_3_PERIOD      64536
#define TIM_3_MS_TO_TICKS(x) (x * 6453.6)

// 168MHz
#define TIM_8_PRESCALER   13
#define TIM_8_PERIOD      65445 * 2
#define TIM_8_MS_TO_TICKS(x) (x * 6453.6 * 2)

// 84Mhz
#define TIM_12_PRESCALER   13
#define TIM_12_PERIOD      64536
#define TIM_12_MS_TO_TICKS(x) (x * 6453.6)

/*
    DEF_TIM(TIM4,  CH2, PB7,  TIM_USE_MC_MOTOR  | TIM_USE_FW_MOTOR,   1, 0), // S1 D(1,3,2)
    DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_MC_MOTOR  | TIM_USE_FW_MOTOR,   1, 0), // S2 D(1,0,2)

    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MC_MOTOR  | TIM_USE_FW_SERVO,   1, 0), // S3 D(1,7,5)
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MC_MOTOR  | TIM_USE_FW_SERVO,   1, 0), // S4 D(1,2,5)
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_MC_MOTOR  | TIM_USE_FW_SERVO,   1, 0), // S5 D(2,4,7)
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_MC_MOTOR  | TIM_USE_FW_SERVO,   1, 0), // S6 D(2,7,7)
    DEF_TIM(TIM12, CH1, PB14, TIM_USE_MC_SERVO  | TIM_USE_FW_SERVO,   1, 0), // S7
    DEF_TIM(TIM12, CH2, PB15, TIM_USE_MC_SERVO  | TIM_USE_FW_SERVO,   1, 0), // S8
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MC_SERVO  | TIM_USE_FW_SERVO,   1, 0), // S9
*/

void ServoOut_Init(void)
{
  TIM_OC_InitTypeDef sConfig;
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_TIM12_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // Configure GPIO
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

  // OUT1
  GPIO_InitStruct.Pin       = GPIO_PIN_7;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // OUT2
  GPIO_InitStruct.Pin       = GPIO_PIN_6;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // OUT3
  GPIO_InitStruct.Pin       = GPIO_PIN_0;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // OUT4
  GPIO_InitStruct.Pin       = GPIO_PIN_1;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // OUT5
  GPIO_InitStruct.Pin       = GPIO_PIN_8;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // OUT6
  GPIO_InitStruct.Pin       = GPIO_PIN_9;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // OUT7
  GPIO_InitStruct.Pin       = GPIO_PIN_14;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // OUT8
  GPIO_InitStruct.Pin       = GPIO_PIN_15;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PWM out config
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.Pulse = 0;

  // Block 1..2 : TIM4
  tim4_handle.Instance = TIM4;
  tim4_handle.Init.Prescaler         = TIM_4_PRESCALER - 1;
  tim4_handle.Init.Period            = TIM_4_PERIOD - 1;
  tim4_handle.Init.ClockDivision     = 0;
  tim4_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  tim4_handle.Init.RepetitionCounter = 0;
  tim4_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&tim4_handle);

  HAL_TIM_PWM_ConfigChannel(&tim4_handle, &sConfig, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&tim4_handle, &sConfig, TIM_CHANNEL_1);

  // Block 3..4 : TIM3
  tim3_handle.Instance = TIM3;
  tim3_handle.Init.Prescaler         = TIM_3_PRESCALER -1;
  tim3_handle.Init.Period            = TIM_3_PERIOD - 1;
  tim3_handle.Init.ClockDivision     = 0;
  tim3_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  tim3_handle.Init.RepetitionCounter = 0;
  tim3_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&tim3_handle);

  HAL_TIM_PWM_ConfigChannel(&tim3_handle, &sConfig, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&tim3_handle, &sConfig, TIM_CHANNEL_4);

  // Block 5..6 : TIM8
  tim8_handle.Instance = TIM8;
  tim8_handle.Init.Prescaler         = TIM_8_PRESCALER - 1;
  tim8_handle.Init.Period            = TIM_8_PERIOD - 1;
  tim8_handle.Init.ClockDivision     = 0;
  tim8_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  tim8_handle.Init.RepetitionCounter = 0;
  tim8_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&tim8_handle);

  HAL_TIM_PWM_ConfigChannel(&tim8_handle, &sConfig, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&tim8_handle, &sConfig, TIM_CHANNEL_4);

  // Block 7..8 : TIM12
  tim12_handle.Instance = TIM12;
  tim12_handle.Init.Prescaler         = TIM_12_PRESCALER -1;
  tim12_handle.Init.Period            = TIM_12_PERIOD - 1;
  tim12_handle.Init.ClockDivision     = 0;
  tim12_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  tim12_handle.Init.RepetitionCounter = 0;
  tim12_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&tim12_handle);

  HAL_TIM_PWM_ConfigChannel(&tim12_handle, &sConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&tim12_handle, &sConfig, TIM_CHANNEL_2);
}

void ServoOut_Set(const enum SERVOOUT_CHANNEL channel, const float value)
{
  switch (channel)
  {
    case SERVOOUT_CHANNEL_1:
      HAL_TIM_PWM_Start(&tim4_handle, TIM_CHANNEL_2);
      __HAL_TIM_SET_COMPARE(&tim4_handle, TIM_CHANNEL_2, TIM_4_MS_TO_TICKS(value));
      break;

    case SERVOOUT_CHANNEL_2:
      HAL_TIM_PWM_Start(&tim4_handle, TIM_CHANNEL_1);
      __HAL_TIM_SET_COMPARE(&tim4_handle, TIM_CHANNEL_1, TIM_4_MS_TO_TICKS(value));
      break;

    case SERVOOUT_CHANNEL_3:
      HAL_TIM_PWM_Start(&tim3_handle, TIM_CHANNEL_3);
      __HAL_TIM_SET_COMPARE(&tim3_handle, TIM_CHANNEL_3, TIM_3_MS_TO_TICKS(value));
      break;

    case SERVOOUT_CHANNEL_4:
      HAL_TIM_PWM_Start(&tim3_handle, TIM_CHANNEL_4);
      __HAL_TIM_SET_COMPARE(&tim3_handle, TIM_CHANNEL_4, TIM_3_MS_TO_TICKS(value));
      break;

    case SERVOOUT_CHANNEL_5:
      HAL_TIM_PWM_Start(&tim8_handle, TIM_CHANNEL_3);
      __HAL_TIM_SET_COMPARE(&tim8_handle, TIM_CHANNEL_3,  TIM_8_MS_TO_TICKS(value));
      break;

    case SERVOOUT_CHANNEL_6:
      HAL_TIM_PWM_Start(&tim8_handle, TIM_CHANNEL_4);
      __HAL_TIM_SET_COMPARE(&tim8_handle, TIM_CHANNEL_4,  TIM_8_MS_TO_TICKS(value));
      break;

    case SERVOOUT_CHANNEL_7:
      HAL_TIM_PWM_Start(&tim12_handle, TIM_CHANNEL_1);
      __HAL_TIM_SET_COMPARE(&tim12_handle, TIM_CHANNEL_1,  TIM_12_MS_TO_TICKS(value));
      break;

    case SERVOOUT_CHANNEL_8:
      HAL_TIM_PWM_Start(&tim12_handle, TIM_CHANNEL_2);
      __HAL_TIM_SET_COMPARE(&tim12_handle, TIM_CHANNEL_2,  TIM_12_MS_TO_TICKS(value));
      break;

    default:
      break;
  }
}

void ServoOut_Stop(const enum SERVOOUT_CHANNEL channel)
{
  switch (channel)
  {
    case SERVOOUT_CHANNEL_1:
      HAL_TIM_PWM_Stop(&tim4_handle, TIM_CHANNEL_2);
      break;

    case SERVOOUT_CHANNEL_2:
      HAL_TIM_PWM_Stop(&tim4_handle, TIM_CHANNEL_1);
      break;

    case SERVOOUT_CHANNEL_3:
      HAL_TIM_PWM_Stop(&tim3_handle, TIM_CHANNEL_3);
      break;

    case SERVOOUT_CHANNEL_4:
      HAL_TIM_PWM_Stop(&tim3_handle, TIM_CHANNEL_4);
      break;

    case SERVOOUT_CHANNEL_5:
      HAL_TIM_PWM_Stop(&tim8_handle, TIM_CHANNEL_3);
      break;

    case SERVOOUT_CHANNEL_6:
      HAL_TIM_PWM_Stop(&tim8_handle, TIM_CHANNEL_4);
      break;

    case SERVOOUT_CHANNEL_7:
      HAL_TIM_PWM_Stop(&tim12_handle, TIM_CHANNEL_1);
      break;

    case SERVOOUT_CHANNEL_8:
      HAL_TIM_PWM_Stop(&tim12_handle, TIM_CHANNEL_2);
      break;

    default:
      break;
  }
}
