 
#include "internal_io.hpp"
#include "diskio.h"
#include "ff.h"

#include "../flash/flash.hpp"
#include "../sdcard/sdcard.hpp"

extern "C"
{
    DWORD get_fattime()
    {
        DWORD result = 0;

        result |= (0 << 0);
        result |= (0 << 5);
        result |= (0 << 11);
        result |= (1 << 16);
        result |= (1 << 21);
        result |= (0 << 25);

        return result;
    }

	int FLASH_disk_status()
	{
		return 0;
	}

	int SD_disk_status()
	{
		return SD_MMC_CARD_STATE_READY;
	}

	int FLASH_disk_initialize()
	{
		flash_init();
		return 0;
	}

	int SD_disk_initialize()
	{
		sd_mmc_init();
		while (SD_MMC_OK != sd_mmc_check(0)) {}
		return 0;
	}

	int FLASH_disk_read(BYTE* buff, DWORD sector, UINT count)
	{
		flash_read(sector * 512, buff, count * 512);
		return 0;
	}

	int SD_disk_read(BYTE* buff, DWORD sector, UINT count)
	{
		sd_mmc_init_read_blocks(0, sector, count);
		sd_mmc_start_read_blocks(buff, count);
		sd_mmc_wait_end_of_read_blocks(false);
		return 0;
	}

	int FLASH_disk_write(const BYTE* buff, DWORD sector, UINT count)
	{
		flash_write(sector * 512, buff, count * 512);
		return 0;
	}

	int SD_disk_write(const BYTE* buff, DWORD sector, UINT count)
	{
		sd_mmc_init_write_blocks(0, sector, count);
		sd_mmc_start_write_blocks(buff, count);
		sd_mmc_wait_end_of_write_blocks(false);
		return 0;
	}

	int FLASH_disk_ioctl(void* buff, BYTE cmd)
	{
		switch (cmd)
		{
		case CTRL_SYNC:
			break;

		case GET_SECTOR_COUNT:
			*(DWORD*)buff = SST26_32MBIT_SIZE / 512; /* 8192 sectors */
			break;

		case GET_SECTOR_SIZE:
			*(WORD*)buff = 512;
			break;

		case GET_BLOCK_SIZE:
			*(DWORD*)buff = 4096;
			break;

		case CTRL_TRIM:
			break;
		}

		return RES_OK;
	}

	int SD_disk_ioctl(void* buff, BYTE cmd)
	{
		switch (cmd)
		{
		case CTRL_SYNC:
			break;

		case GET_SECTOR_COUNT:
			*(DWORD*)buff = sd_mmc_get_capacity(0) * (1024 / 512);
			break;

		case GET_SECTOR_SIZE:
			*(WORD*)buff = 512;
			break;

		case GET_BLOCK_SIZE:
			*(DWORD*)buff = 1;
			break;

		case CTRL_TRIM:
			break;
		}

		return RES_OK;
	}
}