#include <stdio.h>
#include <stdbool.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Retarget.h"
#include "Leds.h"
#include "SpektrumRcIn.h"
#include "ServoOut.h"
#include "FlightControl.h"
#include "Watchdog.h"
#include "I2c1.h"
#include "ImuAhrs.h"
#include "Ticks.h"

void xPortSysTickHandler(void);
void vApplicationTickHook( void );
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
static void MainTask(void* args);
void SystemClock_Config(void);

int main(void)
{
  // Configure low level chip features
  HAL_Init();
  SystemClock_Config();

  // Init Watchdog
  Watchdog_Refresh();
  Watchdog_Init();
  Watchdog_Refresh();

  HAL_InitTick(0);
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  Retarget_Init();
  printf("%cUp!\r\n", 12);

  // Short delay to let the board settle
  HAL_Delay(100);

  // Init LEDs
  Leds_Init();
  Leds_Off(LED_RED);

  // Init I2c1
  I2c1_Init();

  // Init Spektrum
  SpectrumRcIn_Init();

  // Init ServoOut
  ServoOut_Init();

  // Init ImuAhrs
  ImuAhrs_Init();

  // Init FlightControl
  FlightControl_Init();

  // Start main task
  xTaskCreate(MainTask, "MAIN", 256, NULL, 0, NULL);

  // Start scheuler
  vTaskStartScheduler();

  printf("Warning: Scheduler returned\r\n");
  while (true);
}

static void MainTask(void* args)
{
  bool led_state = false;

  while(true)
  {
    // Refresh watchdog
    Watchdog_Refresh();

    // Blink LED
    Leds_Toggle(LED_RED, led_state);
    led_state = !led_state;

    vTaskDelay(100);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
  }
}


void SysTick_Handler(void)
{
  HAL_IncTick();

  // Only run if the scheduler is running
 	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
    xPortSysTickHandler();
  }
}

void vApplicationMallocFailedHook( void )
{
  printf("vApplicationMallocFailedHook\r\n");
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  printf("vApplicationStackOverflowHook\r\n");
}

void vApplicationIdleHook( void )
{
}

void vAssertCalled( uint32_t ulLine, const char *pcFile )
{
  printf("vAssertCalled (Line %lu, File: %s)\r\n", ulLine, pcFile);
}

void vApplicationTickHook( void )
{
}
