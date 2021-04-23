#include <stdio.h>
#include <stdbool.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Retarget.h"

void xPortSysTickHandler(void);
void vApplicationTickHook( void );
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
static void MainTask(void* args);

int main(void)
{
  // Configure low level chip features
  HAL_Init();
  HAL_InitTick(0);
  Retarget_Init();
  printf("%cUp!\r\n", 12);

  // Start main task
  xTaskCreate(MainTask, "MAIN", 1024, NULL, 0, NULL);
  
  // Start scheuler 
  vTaskStartScheduler();
  
  printf("Warning: Scheduler returned\r\n");
  while (1);
}

static void MainTask(void* args)
{
  while(true)
  {
    // Wait for a sensor sample
    vTaskDelay(1000);
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
