#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include "Drivers/SpiSd/SpiSd.h"
#include "Drivers/SpiSd/ff_gen_drv.h"
#include "Drivers/SpiSd/SpiSdDiskIo.h"
#include "Drivers/FatFs/ff.h"

static Diskio_drvTypeDef* spisd_driver;

static char spi_sd_path[4];

static FATFS fs;

static bool TestWriteSdCard(void);

void Fs_Init(void)
{
  Spi3_Init();
  
  spisd_driver = SpiSdDiskIo_GetDriver();
  
  FATFS_LinkDriver(spisd_driver, spi_sd_path);
  
  printf("%.4s\r\n", spi_sd_path);
  
  HAL_Delay(500);
  
  if (f_mount(&fs, "", 0) != FR_OK)
  {
    printf("mount\r\n");
  }
  
  printf("d\r\n");
  
  FRESULT fres;
DWORD fre_clust;
FATFS *pfs;
	/* Check freeSpace space */
	if(f_getfree("", &fre_clust, &pfs) != FR_OK)
		printf("error\r\n");
uint32_t totalSpace, freeSpace;
	totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
  
  printf("%li %li\r\n", totalSpace, freeSpace);
  
  TestWriteSdCard();
}

// Create/open test file t,o check if uSD is present
// This is a more reliable method than looking at the SD card switch
static bool TestWriteSdCard(void)
{
  FIL test_file;
  char test_file_name[] = "TESTFILE.TXT";
  char test_buffer[] = "TESTFILE";
  char test_buffer_read[] = "TESTFILE";
  UINT bytes_written;

  // Open the file and create if not exist
  if (f_open(&test_file, test_file_name, FA_OPEN_APPEND | FA_READ | FA_WRITE) != FR_OK)
  {
    printf("f_open\r\n");
    return false;
  }

  // Try to write to it
  if (f_write(&test_file, test_buffer, sizeof(test_buffer), &bytes_written) != FR_OK)
  {
    printf("f_write\r\n");
    f_unlink(test_file_name);
    return false;
  }

  // Check if we wrote as many bytes as we needed to
  if (sizeof(test_buffer) != bytes_written)
  {
    printf("sizeof\r\n");
    f_unlink(test_file_name);
    return false;
  }

  // Read
  if (f_read(&test_file, test_buffer_read, sizeof(test_buffer_read), &bytes_written) != FR_OK)
  {
    printf("f_read\r\n");
    f_unlink(test_file_name);
    return false;
  }

  if (memcmp(test_buffer, test_buffer_read, sizeof(test_buffer)) != 0)
  {
    f_unlink(test_file_name);
    return false;
  }

  f_close(&test_file);

  // Remove the file
  if (f_unlink(test_file_name) != FR_OK)
  {
    return false;
  }

  return true;
}