/*
 * utils_FLASH_memory.c
 *
 *  Created on: 16 sty 2016
 *      Author: r.rzechowski
 */

/** @addtogroup F4_RR
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include "stm32f4xx_flash.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

/* Private variables ---------------------------------------------------------*/
// Link Key record:
//  - active record flag (0xFF), to shows if record is active or not (1 byte)
//  - link key type (1 byte)
//  - bluetooth device address (6 bytes)
//  - link key (16 bytes)
#define FLASH_LINK_KEY_NUMBER	5
#define FLASH_LINK_KEY_LENGTH	(uint32_t)(1 + BD_ADDR_LEN + LINK_KEY_LEN + 1)
#define FLASH_LINK_KEY_ADDRESS	(uint32_t)(0x08000000 + 1024000 - 1 - FLASH_LINK_KEY_LENGTH * FLASH_LINK_KEY_NUMBER);
/* Private function prototypes -----------------------------------------------*/
uint32_t FlashMemory_GetSector(uint32_t Address);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  FlashMemory_WirteLinkKey
  * 	This will write the data Bluetooth link key at the end of the last sector of
  * 	programmable memory bloc.
  * 	As the flash memory can only be programmed after full sector erase the
  * 	procedure is adding new records only (it is not erasing the block). Duplicated
  * 	addresses are marked as inactive. If there is not free slot left new link key
  * 	is not stored. The impact of such design is that PIN might be required during
  * 	connection.
  * 	The concept was to erase the block with full memory programming or prepare
  * 	dedicated procedure.
  * @param  bd_addr - bluetooth device address
  * @param  link_key - link key associated with the device
  * @param  link_key_type - type of the link key
  * @retval None
  */
void FlashMemory_WirteLinkKey(bd_addr_t bd_addr, link_key_t link_key, link_key_type_t link_key_type) {
	uint32_t address;
	uint32_t address_found  = FLASH_LINK_KEY_ADDRESS;
	uint8_t emptyFound = 0;
	uint8_t bAddrFound = 0;
	uint8_t i, j;
	FLASH_Status status;

	// Unlocks the FLASH control register access
	FLASH_Unlock();
	// Erase last sector
	address = FLASH_LINK_KEY_ADDRESS;
	// Clear pending flags (if any)
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	// Wait for last operation
	status = FLASH_WaitForLastOperation();
	// If it is not success, return error
	if (status != FLASH_COMPLETE) {
		// Lock FLASH
		FLASH_Lock();
		// Return error
		return;
	}
	//Before programming all bytes must be erased. Full block must be erased
	// - no option to erase single byte.
	//FLASH_EraseSector(FlashMemory_GetSector(address), VoltageRange_3);
	//---------------------------------------------------------------------------
	// Write OkPassed code at last word in the flash memory
	// The options can be:
	// - FLASH_ProgramDoubleWord
	// - FLASH_ProgramWord
	// - FLASH_ProgramHalfWord
	// - FLASH_ProgramByte
	//---------------------------------------------------------------------------

	// find the record to replace
	for (j = 0; j < FLASH_LINK_KEY_NUMBER; j++) {
		// empty element found
		if (*(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH + 1) == 0xff) {
			address_found = address + j*FLASH_LINK_KEY_LENGTH;
			emptyFound = 1;
			break;
		}
		// searches for the BT address to check if the link key is not stored yet.
		bAddrFound = 0;
		if (*(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH) == 0xff) {
			bAddrFound = 1;
			for(i = 0; i < BD_ADDR_LEN; i++) {
				if(bd_addr[i] != *(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH + 2 + i)) {
					bAddrFound = 0;
					break;
				}
			}
		}
		// If address was set before record is deactivated
		if (bAddrFound == 1 && (*(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH)) == 0xff) {
			FLASH_ProgramByte((address + j*FLASH_LINK_KEY_LENGTH), 0);
		}
	}

	if (emptyFound == 1) {
		// program Flash
		status = FLASH_COMPLETE;
		address_found++; // this is to leave the record as activated
		FLASH_ProgramByte(address_found, (uint8_t)link_key_type);
		address_found++;
		for (i = 0; i < BD_ADDR_LEN; i++) {
			FLASH_ProgramByte(address_found, bd_addr[i]);
			address_found++;
		}
		for (i = 0; i < LINK_KEY_LEN; i++) {
			FLASH_ProgramByte(address_found, link_key[i]);
			address_found++;
		}
	}
	FLASH_Lock();
}

/**
  * @brief  FlashMemory_ReadLinkKey
  * 	This will write the data Bluetooth link key at the end of the last sector of
  * 	programmable memory bloc.
  * @param  bd_addr - bluetooth device address
  * @param  link_key - buffer for reading link key
  * @param  link_key_type - type of the link key
  * @retval 0 - no found 1 - OK
  */
uint8_t FlashMemory_ReadLinkKey(bd_addr_t bd_addr, link_key_t link_key, link_key_type_t *link_key_type) {
	uint32_t address;
	uint8_t i , j;
	address = FLASH_LINK_KEY_ADDRESS;

	// search the address for link key
	for (j = 0; j < FLASH_LINK_KEY_NUMBER; j++) {
		// record is not empty
		if ((*(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH) == 0xff) &&
				(*(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH + 1)) < 0xff) {
			*link_key_type = *(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH + 1);
			// if address is same
			for(i = 0; i < BD_ADDR_LEN; i++) {
				if(bd_addr[i] != *(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH + 2 + i)) {
					break;
				}
			}
			if (i >= BD_ADDR_LEN) { ///matching address found
				for (i = 0; i < LINK_KEY_LEN; i++) {
					link_key[i] = *(__IO uint8_t*) (address + j*FLASH_LINK_KEY_LENGTH + 2 + BD_ADDR_LEN + i);
				}
				return 1;
			}
		}
	}
	return 0;
}

/**
  * @brief  FlashMemory_GetSector
  * 	The function translates address to the sector in flash memory.
  * @param  The memory address to be translated
  * @retval The selected sector number.
  */
uint32_t FlashMemory_GetSector(uint32_t Address) {
	uint32_t sector = 0;

	if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0)) {
		sector = FLASH_Sector_0;
	} else if ((Address < ADDR_FLASH_SECTOR_2)
			&& (Address >= ADDR_FLASH_SECTOR_1)) {
		sector = FLASH_Sector_1;
	} else if ((Address < ADDR_FLASH_SECTOR_3)
			&& (Address >= ADDR_FLASH_SECTOR_2)) {
		sector = FLASH_Sector_2;
	} else if ((Address < ADDR_FLASH_SECTOR_4)
			&& (Address >= ADDR_FLASH_SECTOR_3)) {
		sector = FLASH_Sector_3;
	} else if ((Address < ADDR_FLASH_SECTOR_5)
			&& (Address >= ADDR_FLASH_SECTOR_4)) {
		sector = FLASH_Sector_4;
	} else if ((Address < ADDR_FLASH_SECTOR_6)
			&& (Address >= ADDR_FLASH_SECTOR_5)) {
		sector = FLASH_Sector_5;
	} else if ((Address < ADDR_FLASH_SECTOR_7)
			&& (Address >= ADDR_FLASH_SECTOR_6)) {
		sector = FLASH_Sector_6;
	} else if ((Address < ADDR_FLASH_SECTOR_8)
			&& (Address >= ADDR_FLASH_SECTOR_7)) {
		sector = FLASH_Sector_7;
	} else if ((Address < ADDR_FLASH_SECTOR_9)
			&& (Address >= ADDR_FLASH_SECTOR_8)) {
		sector = FLASH_Sector_8;
	} else if ((Address < ADDR_FLASH_SECTOR_10)
			&& (Address >= ADDR_FLASH_SECTOR_9)) {
		sector = FLASH_Sector_9;
	} else if ((Address < ADDR_FLASH_SECTOR_11)
			&& (Address >= ADDR_FLASH_SECTOR_10)) {
		sector = FLASH_Sector_10;
	} else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
	{
		sector = FLASH_Sector_11;
	}
	return sector;
}

/**
  * @}
  */
