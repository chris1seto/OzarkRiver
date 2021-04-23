/*
	This file contains the entry point (Reset_Handler) of your firmware project.
	The reset handled initializes the RAM and calls system library initializers as well as
	the platform-specific initializer and the main() function.
*/

#include <stddef.h>
extern void *_estack;

void Reset_Handler();
void Default_Handler();
void NMI_Handler()                    __attribute__ ((weak, alias ("Default_Handler")));
void HardFault_Handler()              __attribute__ ((weak, alias ("Default_Handler")));
void MemManage_Handler()  __attribute__ ((weak, alias ("Default_Handler")));
void BusFault_Handler()  __attribute__ ((weak, alias ("Default_Handler")));
void UsageFault_Handler()  __attribute__ ((weak, alias ("Default_Handler")));
void SVC_Handler()  __attribute__ ((weak, alias ("Default_Handler")));
void DebugMon_Handler()  __attribute__ ((weak, alias ("Default_Handler")));
void PendSV_Handler()  __attribute__ ((weak, alias ("Default_Handler")));
void SysTick_Handler()  __attribute__ ((weak, alias ("Default_Handler")));
void WWDG_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void PVD_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TAMP_STAMP_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void RTC_WKUP_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void FLASH_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void RCC_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void EXTI0_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void EXTI1_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void EXTI2_TSC_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void EXTI3_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void EXTI4_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel1_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel2_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel3_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel4_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel5_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel6_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel7_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void ADC1_2_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USB_HP_CAN_TX_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USB_LP_CAN_RX0_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void CAN_RX1_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void CAN_SCE_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void EXTI9_5_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_CC_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM2_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM3_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM4_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void I2C1_EV_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void I2C1_ER_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void I2C2_EV_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void I2C2_ER_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void SPI1_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void SPI2_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USART1_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USART2_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USART3_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void EXTI15_10_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void RTC_Alarm_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USBWakeUp_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_BRK_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_UP_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_TRG_COM_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_CC_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void ADC3_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void SPI3_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void UART4_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void UART5_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM6_DAC_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void TIM7_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel1_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel2_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel3_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel4_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel5_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void ADC4_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void COMP1_2_3_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void COMP4_5_6_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void COMP7_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USB_HP_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USB_LP_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void USBWakeUp_RMP_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));
void FPU_IRQHandler()  __attribute__ ((weak, alias ("Default_Handler")));


void * g_pfnVectors[98] __attribute__ ((section (".isr_vector"), used)) = 
{
  &_estack,
  &Reset_Handler,
  &NMI_Handler,
  &HardFault_Handler,
  &MemManage_Handler,
  &BusFault_Handler,
  &UsageFault_Handler,
  NULL,
  NULL,
  NULL,
  NULL,
  &SVC_Handler,
  &DebugMon_Handler,
  NULL,
  &PendSV_Handler,
  &SysTick_Handler,
  &WWDG_IRQHandler,
  &PVD_IRQHandler,
  &TAMP_STAMP_IRQHandler,
  &RTC_WKUP_IRQHandler,
  &FLASH_IRQHandler,
  &RCC_IRQHandler,
  &EXTI0_IRQHandler,
  &EXTI1_IRQHandler,
  &EXTI2_TSC_IRQHandler,
  &EXTI3_IRQHandler,
  &EXTI4_IRQHandler,
  &DMA1_Channel1_IRQHandler,
  &DMA1_Channel2_IRQHandler,
  &DMA1_Channel3_IRQHandler,
  &DMA1_Channel4_IRQHandler,
  &DMA1_Channel5_IRQHandler,
  &DMA1_Channel6_IRQHandler,
  &DMA1_Channel7_IRQHandler,
  &ADC1_2_IRQHandler,
  &USB_HP_CAN_TX_IRQHandler,
  &USB_LP_CAN_RX0_IRQHandler,
  &CAN_RX1_IRQHandler,
  &CAN_SCE_IRQHandler,
  &EXTI9_5_IRQHandler,
  &TIM1_BRK_TIM15_IRQHandler,
  &TIM1_UP_TIM16_IRQHandler,
  &TIM1_TRG_COM_TIM17_IRQHandler,
  &TIM1_CC_IRQHandler,
  &TIM2_IRQHandler,
  &TIM3_IRQHandler,
  &TIM4_IRQHandler,
  &I2C1_EV_IRQHandler,
  &I2C1_ER_IRQHandler,
  &I2C2_EV_IRQHandler,
  &I2C2_ER_IRQHandler,
  &SPI1_IRQHandler,
  &SPI2_IRQHandler,
  &USART1_IRQHandler,
  &USART2_IRQHandler,
  &USART3_IRQHandler,
  &EXTI15_10_IRQHandler,
  &RTC_Alarm_IRQHandler,
  &USBWakeUp_IRQHandler,
  &TIM8_BRK_IRQHandler,
  &TIM8_UP_IRQHandler,
  &TIM8_TRG_COM_IRQHandler,
  &TIM8_CC_IRQHandler,
  &ADC3_IRQHandler,
  NULL,
  NULL,
  NULL,
  &SPI3_IRQHandler,
  &UART4_IRQHandler,
  &UART5_IRQHandler,
  &TIM6_DAC_IRQHandler,
  &TIM7_IRQHandler,
  &DMA2_Channel1_IRQHandler,
  &DMA2_Channel2_IRQHandler,
  &DMA2_Channel3_IRQHandler,
  &DMA2_Channel4_IRQHandler,
  &DMA2_Channel5_IRQHandler,
  &ADC4_IRQHandler,
  NULL,
  NULL,
  &COMP1_2_3_IRQHandler,
  &COMP4_5_6_IRQHandler,
  &COMP7_IRQHandler,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  &USB_HP_IRQHandler,
  &USB_LP_IRQHandler,
  &USBWakeUp_RMP_IRQHandler,
  NULL,
  NULL,
  NULL,
  NULL,
  &FPU_IRQHandler,
};

void SystemInit();
void __libc_init_array();
int main();

extern void *_sidata, *_sdata, *_edata;
extern void *_sbss, *_ebss;

void __attribute__((naked, noreturn)) Reset_Handler()
{
	void **pSource, **pDest;
	for (pSource = &_sidata, pDest = &_sdata; pDest != &_edata; pSource++, pDest++)
		*pDest = *pSource;

	for (pDest = &_sbss; pDest != &_ebss; pDest++)
		*pDest = 0;

	SystemInit();
	__libc_init_array();
	(void)main();
	for (;;) ;
}

void __attribute__((naked, noreturn)) Default_Handler()
{
}