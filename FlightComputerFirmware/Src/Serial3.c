#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_usart.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "QueueBuffer.h"
#include "Ticks.h"
#include "SerialInterface.h"
#include "Serial3.h"

static UART_HandleTypeDef uart_handle;
static LL_DMA_InitTypeDef dma_rx_handle;

// Serial DMA buffer
#define DMA_BUFFER_SIZE      256
static uint8_t dma_rx_buffer[DMA_BUFFER_SIZE];
static volatile uint32_t dma_head = 0;

static QueueBuffer_t serial_rx_queue_buffer;
static uint8_t serial_rx_bufer[DMA_BUFFER_SIZE];

static EventGroupHandle_t serial_event_group;
static EventGroupHandle_t serial_interface_event_group;

static uint32_t data_rx_event_count = 0;

enum SERIAL_EVENT
{
  SERIAL_EVENT_NEW_DATA_RX = (1 << 0),
  SERIAL_EVENT_NEW_DATA_TX = (1 << 1),
};

static const char* TAG = "SERIAL3";

static void Serial3Deinit(void);
static bool Serial3Init(const uint32_t baud, const uint32_t new_data_rx_event_count);
static void CopyFromDma(void);
static void Serial3Task(void* arg);

static SerialInterface_t serial_interface =
{
  Serial3Init,
  Serial3Deinit,
  NULL,
  &serial_rx_queue_buffer,
  NULL,
  &serial_interface_event_group,
};

SerialInterface_t* Serial3_GetInterface(void)
{
  return (SerialInterface_t*)&serial_interface;
}

#define SERIAL_MAX_PERIOD   (20 / portTICK_PERIOD_MS)

static void Serial3Deinit(void)
{
}

static bool Serial3Init(const uint32_t baud, const uint32_t new_data_rx_event_count)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // Init data structures
  QueueBuffer_Init(&serial_rx_queue_buffer, serial_rx_bufer, DMA_BUFFER_SIZE);
  serial_event_group = xEventGroupCreate();
  serial_interface_event_group = xEventGroupCreate();

  data_rx_event_count = new_data_rx_event_count;

  // Enable clocks
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  // Configure GPIO
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

  // RX
  GPIO_InitStruct.Pin       = GPIO_PIN_11;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*
    USART3 DMA1
      RX: (Stream 1, Channel 4), 
      TX: (Stream 3, Channel 4), 
  */

  // DMA RX
  dma_rx_handle.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(USART3);
  dma_rx_handle.MemoryOrM2MDstAddress = (uint32_t)dma_rx_buffer;
  dma_rx_handle.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  dma_rx_handle.Mode = LL_DMA_MODE_CIRCULAR;
  dma_rx_handle.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_rx_handle.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_rx_handle.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_rx_handle.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_rx_handle.NbData = DMA_BUFFER_SIZE;
  dma_rx_handle.Channel = LL_DMA_CHANNEL_4;
  dma_rx_handle.Priority = LL_DMA_PRIORITY_VERYHIGH;
  LL_DMA_Init(DMA1, LL_DMA_STREAM_1, &dma_rx_handle);

  // Configure the USART peripheral
  uart_handle.Instance          = USART3;
  uart_handle.Init.BaudRate     = baud;
  uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;
  uart_handle.Init.StopBits     = UART_STOPBITS_1;
  uart_handle.Init.Parity       = UART_PARITY_NONE;
  uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
  uart_handle.Init.Mode         = UART_MODE_RX;

  HAL_UART_Init(&uart_handle);

  __HAL_UART_ENABLE_IT(&uart_handle, UART_IT_IDLE);

  __HAL_UART_ENABLE(&uart_handle);

  HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  // Clear possible faults before enabling reception
  __HAL_UART_CLEAR_PEFLAG(&uart_handle);
  __HAL_UART_CLEAR_NEFLAG(&uart_handle);
  __HAL_UART_CLEAR_OREFLAG(&uart_handle);

  // Start RX
  LL_USART_EnableDMAReq_RX(USART3);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);

  xTaskCreate(Serial3Task, TAG, 500, NULL, 0, NULL);

  return true;
}

void USART3_IRQHandler(void)
{
  BaseType_t HigherPriorityTaskWoken;

  if (__HAL_UART_GET_FLAG(&uart_handle, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&uart_handle);

    xEventGroupSetBitsFromISR(serial_event_group, SERIAL_EVENT_NEW_DATA_RX, &HigherPriorityTaskWoken);
  }
}

static void CopyFromDma(void)
{
  uint32_t end_index;

  // Get the index of the end of the buffer
  end_index = DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);

  // If there is nothing to copy
  // If you let the buffer loop back, you will lose the entire buffer
  if (end_index == dma_head)
  {
    return;
  }

  // Check if the buffer looped over
  if (end_index < dma_head)
  {
    // Copy from the end of the dma buffer
    QueueBuffer_AppendBuffer(&serial_rx_queue_buffer, dma_rx_buffer + dma_head, DMA_BUFFER_SIZE - dma_head);

    // It has. We need to copy from the end of the circ buffer, then again from the start
    // Copy from the start of the dma buffer
    QueueBuffer_AppendBuffer(&serial_rx_queue_buffer, dma_rx_buffer, end_index);
  }
  else
  {
    // Copy from the dma_head to the end_index
    QueueBuffer_AppendBuffer(&serial_rx_queue_buffer, dma_rx_buffer + dma_head, end_index - dma_head);
  }

  // Relocate the DMA head to the end of where we just read
  dma_head = end_index;
}

static void Serial3Task(void* arg)
{
  while (true)
  {
    // We want to fire ever SERIAL_MAX_PERIOD or sooner when an event happens
    // Waiting for event bits or a timeout is our blocking call
    xEventGroupWaitBits(serial_event_group, SERIAL_EVENT_NEW_DATA_RX, false, false, SERIAL_MAX_PERIOD);

    if (xEventGroupGetBits(serial_event_group) & SERIAL_EVENT_NEW_DATA_RX)
    {
      xEventGroupClearBits(serial_event_group, SERIAL_EVENT_NEW_DATA_RX);
    }

    // Copy data in from UART RX
    // Do this whenever we run
    CopyFromDma();

    // If we have enough bytes to fire a new data event
    if (QueueBuffer_Count(&serial_rx_queue_buffer) > data_rx_event_count)
    {
      xEventGroupSetBits(serial_interface_event_group, SERIAL_INTERFACE_EVENT_NEW_DATA_RX);
    }
  }
}