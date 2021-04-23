#include <stdio.h>
#include <stm32f3xx_hal.h>

static UART_HandleTypeDef uart_handle;

// #define UART3_RX_PIN            PB11
// #define UART3_TX_PIN            PB10
void Retarget_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

	// Enable clocks
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();
	
	// Configure GPIO
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin       = GPIO_PIN_10;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// Configure the USART peripheral
	uart_handle.Instance          = USART3;
	uart_handle.Init.BaudRate     = 230400;
	uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;
	uart_handle.Init.StopBits     = UART_STOPBITS_1;
	uart_handle.Init.Parity       = UART_PARITY_NONE;
	uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	uart_handle.Init.Mode         = UART_MODE_TX;

	// Commit the USART
	if (HAL_UART_Init(&uart_handle) != HAL_OK)
	{
		while (1);
	}
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&uart_handle, (uint8_t*)ptr, len, 0xFFFF);
	return len;
}