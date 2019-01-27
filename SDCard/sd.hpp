#pragma once
#ifndef SD_HPP_
#define SD_HPP_

/* Boost Includes */
#include <boost/shared_ptr.hpp>

/* Chimera Includes */
#include <Chimera/spi.hpp>
#include <Chimera/gpio.hpp>

/* FatFS Includes */
#include <SDCard/fatFS/ff.h>
#include <SDCard/fatFS/ff_gen_drv.h>

/* FreeRTOS Includes */
#ifdef USING_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif 


namespace SD
{
	enum {
		BSP_SD_OK = 0x00,
		MSD_OK = 0x00,
		BSP_SD_ERROR = 0x01,
		BSP_SD_TIMEOUT
	};

	typedef struct
	{
		uint8_t  Reserved1 : 2; /* Reserved */
		uint16_t DeviceSize : 12; /* Device Size */
		uint8_t  MaxRdCurrentVDDMin : 3; /* Max. read current @ VDD min */
		uint8_t  MaxRdCurrentVDDMax : 3; /* Max. read current @ VDD max */
		uint8_t  MaxWrCurrentVDDMin : 3; /* Max. write current @ VDD min */
		uint8_t  MaxWrCurrentVDDMax : 3; /* Max. write current @ VDD max */
		uint8_t  DeviceSizeMul : 3; /* Device size multiplier */
	} struct_v1;

	typedef struct
	{
		uint8_t  Reserved1 : 6; /* Reserved */
		uint32_t DeviceSize : 22; /* Device Size */
		uint8_t  Reserved2 : 1; /* Reserved */
	} struct_v2;

	typedef struct
	{
		/* Card Specific Data: CSD Register */
		/* Header part */
		uint8_t  CSDStruct : 2; /* CSD structure */
		uint8_t  Reserved1 : 6; /* Reserved */
		uint8_t  TAAC : 8; /* Data read access-time 1 */
		uint8_t  NSAC : 8; /* Data read access-time 2 in CLK cycles */
		uint8_t  MaxBusClkFrec : 8; /* Max. bus clock frequency */
		uint16_t CardComdClasses : 12; /* Card command classes */
		uint8_t  RdBlockLen : 4; /* Max. read data block length */
		uint8_t  PartBlockRead : 1; /* Partial blocks for read allowed */
		uint8_t  WrBlockMisalign : 1; /* Write block misalignment */
		uint8_t  RdBlockMisalign : 1; /* Read block misalignment */
		uint8_t  DSRImpl : 1; /* DSR implemented */

							  /* v1 or v2 struct */
		union csd_version {
			struct_v1 v1;
			struct_v2 v2;
		} version;

		uint8_t  EraseSingleBlockEnable : 1; /* Erase single block enable */
		uint8_t  EraseSectorSize : 7; /* Erase group size multiplier */
		uint8_t  WrProtectGrSize : 7; /* Write protect group size */
		uint8_t  WrProtectGrEnable : 1; /* Write protect group enable */
		uint8_t  Reserved2 : 2; /* Reserved */
		uint8_t  WrSpeedFact : 3; /* Write speed factor */
		uint8_t  MaxWrBlockLen : 4; /* Max. write data block length */
		uint8_t  WriteBlockPartial : 1; /* Partial blocks for write allowed */
		uint8_t  Reserved3 : 5; /* Reserved */
		uint8_t  FileFormatGrouop : 1; /* File format group */
		uint8_t  CopyFlag : 1; /* Copy flag (OTP) */
		uint8_t  PermWrProtect : 1; /* Permanent write protection */
		uint8_t  TempWrProtect : 1; /* Temporary write protection */
		uint8_t  FileFormat : 2; /* File Format */
		uint8_t  Reserved4 : 2; /* Reserved */
		uint8_t  crc : 7; /* Reserved */
		uint8_t  Reserved5 : 1; /* always 1*/
	} SD_CSD;

	typedef struct
	{
		/* Card Identification Data: CID Register */
		__IO uint8_t  ManufacturerID; /* ManufacturerID */
		__IO uint16_t OEM_AppliID; /* OEM/Application ID */
		__IO uint32_t ProdName1; /* Product Name part1 */
		__IO uint8_t  ProdName2; /* Product Name part2*/
		__IO uint8_t  ProdRev; /* Product Revision */
		__IO uint32_t ProdSN; /* Product Serial Number */
		__IO uint8_t  Reserved1; /* Reserved1 */
		__IO uint16_t ManufactDate; /* Manufacturing Date */
		__IO uint8_t  CID_CRC; /* CID CRC */
		__IO uint8_t  Reserved2; /* always 1 */
	} SD_CID;

	typedef struct
	{
		SD_CSD Csd;
		SD_CID Cid;
		uint32_t CardCapacity; /*!< Card Capacity */
		uint32_t CardBlockSize; /*!< Card Block Size */
		uint32_t LogBlockNbr; /*!< Specifies the Card logical Capacity in blocks   */
		uint32_t LogBlockSize; /*!< Specifies logical block size in bytes           */
	} SD_CardInfo;

	const unsigned int SD_BLOCK_SIZE = 0x200;
	const unsigned int SD_PRESENT = 0x01;
	const unsigned int SD_NOT_PRESENT = 0x00;
	const unsigned int SD_DATATIMEOUT = 100000000;

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

		FRESULT initialize(bool formatDrive = false, uint32_t formatOptions = FM_FAT32);
		void deInitialize();
		FRESULT format(uint32_t FA_options);
		FRESULT fopen(const char* filename, uint32_t options, FIL& file_ptr);
		FRESULT read(FIL& file_ptr, uint8_t* data_out, size_t length, uint32_t& bytesRead);
		FRESULT write(FIL& file_ptr, uint8_t* data_in, size_t length, uint32_t& bytesWritten);
		FRESULT fclose(FIL& file_ptr);

		SDCard(const uint8_t& spiChannel, Chimera::GPIO::Port port, uint8_t pin);
		~SDCard();

	private:
		Chimera::SPI::SPIClass_uPtr spi;
		Chimera::GPIO::GPIOClass_uPtr ssPin;

		SemaphoreHandle_t spiTXRXWakeup;	/**< Unblocks the SDCard thread once the transmission/reception is complete */

		FATFS SDFatFs;
		//FIL File;
		char SDPath[4]; /* SD Disk Logical Drive Path */
		uint8_t workBuffer[_MAX_SS];

		volatile DSTATUS STATUS;

		struct SD_CmdAnswer
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
		/**
		* @brief  Initializes the SD/SD communication.
		* @param  None
		* @retval The SD Response:
		*         - MSD_ERROR: Sequence failed
		*         - MSD_OK: Sequence succeed
		*/
		uint8_t BSP_SD_Init(void);

		/**
		* @brief  Reads block(s) from a specified address in the SD card, in polling mode.
		* @param  pData: Pointer to the buffer that will contain the data to transmit
		* @param  ReadAddr: Address from where data is to be read. The address is counted
		*                   in blocks of 512bytes
		* @param  NumOfBlocks: Number of SD blocks to read
		* @param  Timeout: This parameter is used for compatibility with BSP implementation
		* @retval SD status
		*/
		uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);

		/**
		* @brief  Writes block(s) to a specified address in the SD card, in polling mode.
		* @param  pData: Pointer to the buffer that will contain the data to transmit
		* @param  WriteAddr: Address from where data is to be written. The address is counted
		*                   in blocks of 512bytes
		* @param  NumOfBlocks: Number of SD blocks to write
		* @param  Timeout: This parameter is used for compatibility with BSP implementation
		* @retval SD status
		*/
		uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);

		/**
		 * @brief  Erases the specified memory area of the given SD card.
		 * @param  StartAddr: Start address in Blocks (Size of a block is 512bytes)
		 * @param  EndAddr: End address in Blocks (Size of a block is 512bytes)
		 * @retval SD status
		 **/
		uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);

		/**
		* @brief  Returns the SD status.
		* @param  None
		* @retval The SD status.
		*/
		uint8_t BSP_SD_GetCardState(void);

		/**
		* @brief  Returns information about specific card.
		* @param  pCardInfo: Pointer to a SD_CardInfo structure that contains all SD
		*         card information.
		* @retval The SD Response:
		*         - MSD_ERROR: Sequence failed
		*         - MSD_OK: Sequence succeed
		*/
		uint8_t BSP_SD_GetCardInfo(SD_CardInfo *pCardInfo);

		void    SD_IO_Init(void);
		void    SD_IO_CSState(uint8_t state);
		void    SD_IO_WriteReadData(uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
		uint8_t SD_IO_WriteByte(uint8_t Data);

		/**
		* @brief  Reads the SD card CID register.
		*         Reading the contents of the CID register in SPI mode is a simple
		*         read-block transaction.
		* @param  Cid: pointer on an CID register structure
		* @retval SD status
		*/
		uint8_t SD_GetCIDRegister(SD_CID* Cid);

		/**
		* @brief  Reads the SD card SCD register.
		*         Reading the contents of the CSD register in SPI mode is a simple
		*         read-block transaction.
		* @param  Csd: pointer on an SCD register structure
		* @retval SD status
		*/
		uint8_t SD_GetCSDRegister(SD_CSD* Csd);

		/**
		* @brief  Gets the SD card data response and check the busy flag.
		* @param  None
		* @retval The SD status: Read data response xxx0<status>1
		*         - status 010: Data accepted
		*         - status 101: Data rejected due to a crc error
		*         - status 110: Data rejected due to a Write error.
		*         - status 111: Data rejected due to other error.
		*/
		uint8_t SD_GetDataResponse(void);

		/**
		* @brief  Put the SD in Idle state.
		* @param  None
		* @retval SD status
		*/
		uint8_t SD_GoIdleState(void);

		/**
		* @brief  Sends 5 bytes command to the SD card and get response
		* @param  Cmd: The user expected command to send to SD card.
		* @param  Arg: The command argument.
		* @param  Crc: The CRC.
		* @param  Answer: SD_ANSWER_NOT_EXPECTED or SD_ANSWER_EXPECTED
		* @retval SD status
		*/
		SD_CmdAnswer SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Answer);

		/**
		* @brief  Waits a data from the SD card
		* @param  data : Expected data from the SD card
		* @retval BSP_SD_OK or BSP_SD_TIMEOUT
		*/
		uint8_t SD_WaitData(uint8_t data);

		/**
		* @brief  Waits a data until a value different from SD_DUMMY_BITE
		* @param  None
		* @retval the value read
		*/
		uint8_t SD_ReadData(void);


		void SPI_Init();
	};
	typedef boost::shared_ptr<SDCard> SDCard_sPtr;
}




#endif