#ifndef USB_H
#define USB_H

#include <stm32f4xx_hal.h>
#include "Usb/usbd_desc.h"
#include "Usb/usbd_cdc.h"
#include "Usb/usbd_core.h"
#include "Usb/usbd_cdc_if.h"
#include "Usb/usbd_def.h"

void UsbInit(void);

USBD_HandleTypeDef USBD_Device;

#endif