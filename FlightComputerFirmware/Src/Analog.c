#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include "Analog.h"

// Peripheral handles
static DMA_HandleTypeDef dma_handle;
static ADC_HandleTypeDef adc_handle;

// DMA Sample buffer
// This buffer is divided into two parts, so we cleverly treat it as a double buffer
#define ADC_SAMPLE_BUFFER_SIZE    32*2  // Buffer of size 32, times two because double buffer
#define TEMP_COUNT          16    // 16 samples in each buffer
#define VSENSE_COUNT        16    // 16 samples in each buffer
static volatile uint16_t adc_buffer[ADC_SAMPLE_BUFFER_SIZE];

// ADC calibration slope
static float t_slope;

// Temperature sensor trimming values
// Defined DocID022152 Rev 8 page 138
#define TS_CAL1_T    30.0f
#define TS_CAL2_T    110.0f
#define TS_CAL1      (uint16_t*)(0x1FFF7A2C)
#define TS_CAL2      (uint16_t*)(0x1FFF7A2E)

// Init BSP peripherals
void Analog_Init(void)
{
  // Pa1 Vsense
  GPIO_InitTypeDef GPIO_InitStruct;
  ADC_ChannelConfTypeDef sConfig;

  // Precalculate the temperature curve slope
  t_slope = (TS_CAL2_T - TS_CAL1_T) / (*TS_CAL2 - *TS_CAL1);

  // Enable Clocks
  __GPIOA_CLK_ENABLE();
  __ADC1_CLK_ENABLE();
  __DMA2_CLK_ENABLE();

  // Configure ADC output pin
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  // PA1 ADC1_IN1
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure ADC
  adc_handle.Instance                   = ADC1;
  adc_handle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV8;
  adc_handle.Init.Resolution            = ADC_RESOLUTION_12B;
  adc_handle.Init.ScanConvMode          = ENABLE;
  adc_handle.Init.ContinuousConvMode    = ENABLE;
  adc_handle.Init.DiscontinuousConvMode = DISABLE;
  adc_handle.Init.NbrOfDiscConversion   = 0;
  adc_handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIG_EDGE_NONE;
  adc_handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  adc_handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  adc_handle.Init.NbrOfConversion       = 2;
  adc_handle.Init.DMAContinuousRequests = ENABLE;
  adc_handle.Init.EOCSelection          = DISABLE;
  HAL_ADC_Init(&adc_handle);

  // Configure in channels
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfig.Offset       = 0;

  // V+ VSense
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = 1;
  HAL_ADC_ConfigChannel(&adc_handle, &sConfig);

  // uC temperature sensor
  // HAL automatically configures the temperature sensor
  sConfig.Channel      = ADC_CHANNEL_16;
  sConfig.Rank         = 2;
  HAL_ADC_ConfigChannel(&adc_handle, &sConfig);

  // Configure DMA
  // ADC1 DMA2 Stream4, channel 0
  dma_handle.Instance            = DMA2_Stream4;
  dma_handle.Init.Channel       = DMA_CHANNEL_0;
  dma_handle.Init.Direction     = DMA_PERIPH_TO_MEMORY;
  dma_handle.Init.PeriphInc     = DMA_PINC_DISABLE;
  dma_handle.Init.MemInc        = DMA_MINC_ENABLE;
  dma_handle.Init.PeriphDataAlignment  = DMA_PDATAALIGN_HALFWORD;
  dma_handle.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
  dma_handle.Init.Mode             = DMA_CIRCULAR;
  dma_handle.Init.Priority         = DMA_PRIORITY_HIGH;
  dma_handle.Init.FIFOMode         = DMA_FIFOMODE_DISABLE;
  dma_handle.Init.FIFOThreshold    = DMA_FIFO_THRESHOLD_HALFFULL;
  dma_handle.Init.MemBurst         = DMA_MBURST_SINGLE;
  dma_handle.Init.PeriphBurst      = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&dma_handle);

  __HAL_LINKDMA(&adc_handle, DMA_Handle, dma_handle);

  // Enable interrupts
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0xf, 0xf);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

  // Start measuring
  HAL_ADC_Start_DMA(&adc_handle, (uint32_t*)adc_buffer, ADC_SAMPLE_BUFFER_SIZE);
}

static void ConvertAdcValues(const bool first_half)
{
}

// Handle BSP analog sense
void DMA2_Stream4_IRQHandler(void)
{
  // Handle half transfer complete
  if(__HAL_DMA_GET_FLAG(&dma_handle, DMA_FLAG_HTIF0_4))
  {
    __HAL_DMA_CLEAR_FLAG(&dma_handle, DMA_FLAG_HTIF0_4);

    // Convert the first half of the buffer
    ConvertAdcValues(true);
  }

  // Handle transfer complete
  if(__HAL_DMA_GET_FLAG(&dma_handle, DMA_FLAG_TCIF0_4))
  {
    __HAL_DMA_CLEAR_FLAG(&dma_handle, DMA_FLAG_TCIF0_4);

    // Transfer is complete, process the latter half of the buffer
    ConvertAdcValues(false);
  }

  // Handle transfer error
  if(__HAL_DMA_GET_FLAG(&dma_handle, DMA_FLAG_TEIF0_4))
  {
    __HAL_DMA_CLEAR_FLAG(&dma_handle, DMA_FLAG_TEIF0_4);
  }

  // FIFO error
  if(__HAL_DMA_GET_FLAG(&dma_handle, DMA_FLAG_FEIF0_4))
  {
    __HAL_DMA_CLEAR_FLAG(&dma_handle, DMA_FLAG_FEIF0_4);
  }
}

static float ConvertStm32TemperatureTicks(const float adc_ticks)
{
  /*
    Ref:
      * DocID022152 Rev 8, page 39, 138
      * DocID022101 Rev 3

    The temperature sensor in the STM32 is a low accuracy device,
    though it can be calibrated using TS_CAL1 and TS_CAL2
  */

  // Calculate temperature
  return (t_slope * (adc_ticks - *TS_CAL1)) + TS_CAL1_T;
}