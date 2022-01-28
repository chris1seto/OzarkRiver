#include <string.h>
#include "ff_gen_drv.h"
#include "SpiSdDiskIo.h"
#include "SpiSd.h"

static volatile DSTATUS Stat = STA_NOINIT;

static DSTATUS USER_initialize (BYTE pdrv);
static DSTATUS USER_status (BYTE pdrv);
static DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
static DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);  
static DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);

static const Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read, 
  USER_write,
  USER_ioctl,
};

Diskio_drvTypeDef* SpiSdDiskIo_GetDriver(void)
{
  return (Diskio_drvTypeDef*)&USER_Driver;
}

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
static DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
	return SD_disk_initialize(pdrv);
}
 
/**
  * @brief  Gets Disk Status 
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
static DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
	return SD_disk_status(pdrv);
}

/**
  * @brief  Reads Sector(s) 
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
static DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
	return SD_disk_read(pdrv, buff, sector, count);
}

/**
  * @brief  Writes Sector(s)  
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
static DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{ 
	return SD_disk_write(pdrv, buff, sector, count);
}


/**
  * @brief  I/O control operation  
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
static DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
	return SD_disk_ioctl(pdrv, cmd, buff);
}
