#ifndef __FF_GEN_DRV_H
#define __FF_GEN_DRV_H

#include <boost/function.hpp>
#include "diskio.h"
#include "ff.h"
#include "stdint.h"
	
/* Old structure before "boost-ifying" everything */
// typedef struct
// {
//   DSTATUS (*disk_initialize) (BYTE);                     /*!< Initialize Disk Drive                     */
//   DSTATUS (*disk_status)     (BYTE);                     /*!< Get Disk Status                           */
//   DRESULT (*disk_read)       (BYTE, BYTE*, DWORD, UINT);       /*!< Read Sector(s)                            */
// #if _USE_WRITE == 1
//   DRESULT (*disk_write)      (BYTE, const BYTE*, DWORD, UINT); /*!< Write Sector(s) when _USE_WRITE = 0       */
// #endif /* _USE_WRITE == 1 */
// #if _USE_IOCTL == 1
//   DRESULT (*disk_ioctl)      (BYTE, BYTE, void*);              /*!< I/O control operation when _USE_IOCTL = 1 */
// #endif /* _USE_IOCTL == 1 */
// 
// }Diskio_drvTypeDef;

/* New structure. This allows for the use of boost::bind in the SDCard class while 
   still maintaining compatibility with the old code. */
typedef struct
{
	boost::function<DSTATUS(BYTE)> disk_initialize;
	boost::function<DSTATUS(BYTE)> disk_status;
	boost::function<DRESULT(BYTE, BYTE*, DWORD, UINT)> disk_read;

	#if _USE_WRITE == 1
	boost::function<DRESULT(BYTE, const BYTE*, DWORD, UINT)> disk_write;
	#endif

	#if _USE_IOCTL == 1
	boost::function <DRESULT(BYTE, BYTE, void*)> disk_ioctl;
	#endif

} Diskio_drvTypeDef;

typedef struct
{
  uint8_t                 is_initialized[_VOLUMES];
  const Diskio_drvTypeDef *drv[_VOLUMES];
  uint8_t                 lun[_VOLUMES];
  volatile uint8_t        nbr;

}Disk_drvTypeDef;

uint8_t FATFS_LinkDriver(const Diskio_drvTypeDef *drv, char *path);
uint8_t FATFS_UnLinkDriver(char *path);
uint8_t FATFS_LinkDriverEx(const Diskio_drvTypeDef *drv, char *path, BYTE lun);
uint8_t FATFS_UnLinkDriverEx(char *path, BYTE lun);
uint8_t FATFS_GetAttachedDriversNbr(void);


#endif /* __FF_GEN_DRV_H */