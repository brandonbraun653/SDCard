#pragma once
#ifndef SD_H_
#define SD_H_

#include <Thor\include\config.hpp>
#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif

/* FatFS Includes */
#include "fatFS/ff.h"
#include "fatFS/ff_gen_drv.h"

/* Thor Includes */
#include <Thor\include/definitions.hpp>
#include <Thor\include/spi.hpp>

/* Boost Includes */
#include <boost/shared_ptr.hpp>

/* FreeRTOS Includes */
#ifdef USING_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif 

using namespace Thor::Definitions::SPI;
using namespace Thor::Peripheral::SPI;

class SDCard;
typedef boost::shared_ptr<SDCard> SDCard_sPtr;

enum SDStatus
{
	SD_OK,
	SD_WRITE_ERROR,
	SD_READ_ERROR,
	SD_FAILED_INITIALIZATION,
	SD_FILE_NONEXISTANT
};

class SDCard
{
public:

	FRESULT initialize(bool formatDrive=false, uint32_t formatOptions=FM_FAT32);
	void deInitialize();
	FRESULT format(uint32_t FA_options);
	FRESULT fopen(const char* filename, uint32_t options, FIL& file_ptr);
	FRESULT read(FIL& file_ptr, uint8_t* data_out, size_t length, uint32_t& bytesRead);
	FRESULT write(FIL& file_ptr, uint8_t* data_in, size_t length, uint32_t& bytesWritten);
	FRESULT fclose(FIL& file_ptr);

	SDCard(SPIClass_sPtr spi_instance);
	~SDCard();

private:
	SPIClass_sPtr spi;

	FATFS SDFatFs;
	//FIL File;
	char SDPath[4]; /* SD Disk Logical Drive Path */
	uint8_t workBuffer[_MAX_SS];

	volatile DSTATUS STATUS;

	struct SD_CmdAnswer_typedef
	{
		uint8_t r1;
		uint8_t r2;
		uint8_t r3;
		uint8_t r4;
		uint8_t r5;
	};

	/*----------------------------------
	 * Low Level Driver Interface for FatFS
	 *---------------------------------*/
	Diskio_drvTypeDef Driver;

	/* Functions to be paired with the Driver object for low level access */
	DSTATUS SD_Initialize(BYTE lun);
	DSTATUS SD_Status(BYTE lun);
	DRESULT SD_Read(BYTE lun, BYTE *buff, DWORD sector, UINT count);
	#if _USE_WRITE == 1
	DRESULT SD_Write(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
	#endif
	#if _USE_IOCTL == 1
	DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff);
	#endif

	/*----------------------------------
	* Initialization/Driver Functions
	*---------------------------------*/
	uint8_t BSP_SD_Init(void);
	uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
	uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
	uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
	uint8_t BSP_SD_GetCardState(void);
	uint8_t BSP_SD_GetCardInfo(Libraries::SD::SD_CardInfo *pCardInfo);

	void    SD_IO_Init(void);
	void    SD_IO_CSState(uint8_t state);
	void    SD_IO_WriteReadData(uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
	uint8_t SD_IO_WriteByte(uint8_t Data);

	uint8_t SD_GetCIDRegister(Libraries::SD::SD_CID* Cid);
	uint8_t SD_GetCSDRegister(Libraries::SD::SD_CSD* Csd);
	uint8_t SD_GetDataResponse(void);
	uint8_t SD_GoIdleState(void);
	SD_CmdAnswer_typedef SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Answer);
	uint8_t SD_WaitData(uint8_t data);
	uint8_t SD_ReadData(void);


	void SPIx_ReInit();
};

#endif