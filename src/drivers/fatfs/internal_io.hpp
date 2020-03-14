#ifndef __INTERNAL_IO_H__
#define __INTERNAL_IO_H__

#include "ff.h"

#ifdef __cplusplus
extern "C" {
#endif

int FLASH_disk_status();
int SD_disk_status();

int FLASH_disk_initialize();
int SD_disk_initialize();

int FLASH_disk_read(BYTE* buff, DWORD sector, UINT count);
int SD_disk_read(BYTE* buff, DWORD sector, UINT count);

int FLASH_disk_write(const BYTE* buff, DWORD sector, UINT count);
int SD_disk_write(const BYTE* buff, DWORD sector, UINT count);

int FLASH_disk_ioctl(void* buff, BYTE cmd);
int SD_disk_ioctl(void* buff, BYTE cmd);

#ifdef __cplusplus
}
#endif
#endif // __INTERNAL_IO_H__