/*
 * spiflash.h
 *
 *  Created on: Mar 21, 2023
 *      Author: anh
 */

#ifndef COMPONENTS_SPIFLASH_H_
#define COMPONENTS_SPIFLASH_H_

#include "component_config.h"
#if ENABLE_COMPONENT_SPIFLASH

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "periph/spi.h"

#define SPIFLASH_CSPIN_EN

typedef struct {
	uint16_t Code;
	uint16_t PageSize;
	uint32_t PageCount;
	uint32_t SectorSize;
	uint32_t SectorCount;
	uint32_t BlockSize;
	uint32_t BlockCount;
	uint32_t Capacity;
	uint8_t	 SR1;
	uint8_t	 SR2;
	uint8_t	 SR3;
}flash_t;

extern flash_t flash_properties;
extern volatile uint8_t  dma_flash_tx_flag;
extern volatile uint8_t  dma_flash_rx_flag;

class SPIFLASH {
	public:
#ifdef SPIFLASH_CSPIN_EN
		SPIFLASH(GPIO_TypeDef *CSPort, uint16_t CSPin);
#endif
		SPIFLASH(void);
		uint32_t ReadID(void);
		uint32_t Init(spi_t spi);

		void EraseSector(uint32_t Sector_Address);
		void EraseBlock(uint32_t Block_Address);
		void EraseChip(void);

		void ProtectedChip(void);
		void UnProtectedChip(void);

		void ReadBytes(uint32_t ReadAddress, uint8_t *Data, uint16_t NumByteRead = 256);
		void WriteBytes(uint32_t WriteAddress, uint8_t *Data, uint16_t NumByteWrite = 256);

	private:
#ifdef SPIFLASH_CSPIN_EN
		GPIO_TypeDef *_csport;
		uint16_t _cspin;
		void CS_Active(void);
		void CS_Idle(void);
#endif
		spi_t _spi;
		void Reset(void);
		uint8_t ReadStatusRegister(uint8_t Register_Number);
		void WriteEnable(void);
		void WriteDisable(void);
		void WaitWriteEnd(void);
		void WriteStatusRegister(uint8_t Register_Number, uint8_t Command);
		void SetAddress(uint32_t Address);
};


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_SPIFLASH */

#endif /* COMPONENTS_SPIFLASH_H_ */
