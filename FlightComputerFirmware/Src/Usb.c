#include <stm32f4xx_hal.h>
#include "Usb/usbd_desc.h"
#include "Usb/usbd_cdc.h"
#include "Usb/usbd_core.h"
#include "Usb/usbd_cdc_if.h"
#include "Usb/usbd_def.h"

extern USBD_HandleTypeDef USBD_Device;
extern PCD_HandleTypeDef hpcd;

void UsbInit(void)
{
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
  USBD_Start(&USBD_Device);
}

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}