/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "internal_io.hpp"
#include "../sdcard/sd_mmc_protocol.hpp"

/* Definitions of physical drive number for each drive */
#define DEV_FLASH	0	/* Map Flash chip to physical drive 0 */
#define DEV_SD		1	/* Map MMC/SD card to physical drive 1 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv)
	{
	case DEV_FLASH:
		result = FLASH_disk_status();
		return RES_OK;

	case DEV_SD:
		result = SD_disk_status();

		switch (result)
		{
		case SD_MMC_CARD_STATE_READY:
			stat = RES_OK;
			break;

		case SD_MMC_CARD_STATE_DEBOUNCE:
		case SD_MMC_CARD_STATE_INIT:
			stat = STA_NOINIT;
			break;
		
		case SD_MMC_CARD_STATE_UNUSABLE:
		case SD_MMC_CARD_STATE_NO_CARD:
			stat = STA_NODISK;
			break;
		
		default:
			return STA_NODISK;
		}
		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_FLASH:
		result = FLASH_disk_initialize();
		return RES_OK;

	case DEV_SD:
		result = SD_disk_initialize();
		return RES_OK;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_FLASH:
		// translate the arguments here

		result = FLASH_disk_read(buff, sector, count);

		// translate the reslut code here

		return RES_OK;

	case DEV_SD:
		// translate the arguments here

		result = SD_disk_read(buff, sector, count);

		// translate the reslut code here

		return RES_OK;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive number to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_FLASH:
		// translate the arguments here

		result = FLASH_disk_write(buff, sector, count);

		// translate the reslut code here

		return RES_OK;

	case DEV_SD:
		// translate the arguments here

		result = SD_disk_write(buff, sector, count);

		// translate the reslut code here

		return RES_OK;
	}

	return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_FLASH:
		result = FLASH_disk_ioctl(buff, cmd);
		// Process of the command for the Flash chip

		return RES_OK;

	case DEV_SD:
		result = SD_disk_ioctl(buff, cmd);
		// Process of the command for the MMC/SD card

		return RES_OK;
	}

	return RES_PARERR;
}