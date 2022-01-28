#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "Drivers/SpiSd/SpiSd.h"
#include "Drivers/SpiSd/ff_gen_drv.h"
#include "Drivers/SpiSd/SpiSdDiskIo.h"
#include "Drivers/FatFs/ff.h"

static Diskio_drvTypeDef* spisd_driver;

static char spi_sd_path[4];

void Fs_Init(void)
{
  Spi3_Init();
  
  spisd_driver = SpiSdDiskIo_GetDriver();
  
  FATFS_LinkDriver(spisd_driver, spi_sd_path);
}