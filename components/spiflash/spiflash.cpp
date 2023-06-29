/*
 * spiflash.cpp
 *
 *  Created on: Mar 21, 2023
 *      Author: anh
 */
#include "component_config.h"
#if ENABLE_COMPONENT_SPIFLASH


#include "spiflash.h"
#include "periph/gpio.h"
#include "periph/systick.h"
#include "stdlib.h"
#include "string.h"

#define CMD_WRITE_DISABLE  0x04
#define CMD_WRITE_ENABLE   0x06
#define CMD_SECTOR_ERASE   0x20
#define CMD_BLOCK_ERASE    0xD8
#define CMD_CHIP_ERASE     0xC7
#define CMD_ENABLE_RESET   0x66
#define CMD_RESET   	   0x99
#define CMD_FAST_READ      0x0B
#define CMD_PAGE_PROGRAMM  0x02
#define CMD_GET_JEDEC_ID   0x9F
#define CMD_READ_STATUS_1  0x05
#define CMD_READ_STATUS_2  0x35
#define CMD_READ_STATUS_3  0x15
#define CMD_WRITE_STATUS   0x50
#define CMD_WRITE_STATUS_1 0x01
#define CMD_WRITE_STATUS_2 0x31
#define CMD_WRITE_STATUS_3 0x11
#define STATUS_REG_BUSY    0x01

flash_t flash_properties;
volatile uint8_t  dma_flash_tx_flag = 0;
volatile uint8_t  dma_flash_rx_flag = 0;

#ifdef SPIFLASH_CSPIN_EN
SPIFLASH::SPIFLASH(GPIO_TypeDef *CSPort, uint16_t CSPin){
	_csport = CSPort;
	_cspin = CSPin;
}
#endif

SPIFLASH::SPIFLASH(void){}

#ifdef SPIFLASH_CSPIN_EN
void SPIFLASH::CS_Active(void){
	gpio_reset(_csport, _cspin);
}
void SPIFLASH::CS_Idle(void){
	gpio_set(_csport, _cspin);
}
#endif

void SPIFLASH::WriteEnable(void) {
#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

	uint32_t dt = CMD_WRITE_ENABLE;
	_spi -> transmit((uint32_t)&dt, 1);

#ifdef SPIFLASH_CSPIN_EN
	 CS_Idle();
#endif
}

void SPIFLASH::WriteDisable(void) {
#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif
	uint32_t dt = CMD_WRITE_DISABLE;
	_spi -> transmit((uint32_t)&dt, 1);

#ifdef SPIFLASH_CSPIN_EN
	 CS_Idle();
#endif
}

uint8_t SPIFLASH::ReadStatusRegister(uint8_t Register_Number){
	uint8_t status = 0;
	uint8_t dt = 0U;

#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

	switch(Register_Number){
		case 1:
			dt = CMD_READ_STATUS_1;
			_spi -> transmit((uint32_t)&dt, 1);
			_spi -> receive((uint32_t)&status, 1);
			flash_properties.SR1 = status;
		break;
		case 2:
			dt = CMD_READ_STATUS_2;
			_spi -> transmit((uint32_t)&dt, 1);
			_spi -> receive((uint32_t)&status, 1);
			flash_properties.SR2 = status;
		break;
		case 3:
			dt = CMD_READ_STATUS_3;
			_spi -> transmit((uint32_t)&dt, 1);
			_spi -> receive((uint32_t)&status, 1);
			flash_properties.SR3 = status;
		break;
	}

#ifdef SPIFLASH_CSPIN_EN
	 CS_Idle();
#endif

     return status;
}

void SPIFLASH::WriteStatusRegister(uint8_t StatusRegisterNum, uint8_t cmd){
	uint8_t dt = 0U;

    WaitWriteEnd();
    WriteEnable();

#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

    switch(StatusRegisterNum){
    	case 1:
			dt = CMD_WRITE_STATUS_1;
    	break;
    	case 2:
    		dt = CMD_WRITE_STATUS_2;
    	break;
    	case 3:
    		dt = CMD_WRITE_STATUS_3;
    	break;
    }
    _spi -> transmit((uint32_t)&dt, 1);
    _spi -> transmit((uint32_t)&cmd, 1);

#ifdef SPIFLASH_CSPIN_EN
	 CS_Idle();
#endif

    WriteDisable();
    WaitWriteEnd();
}

void SPIFLASH::ProtectedChip(void){
	WriteStatusRegister(1, 0xFF);
}
void SPIFLASH::UnProtectedChip(void){
	WriteStatusRegister(1, 0x00);
}

void SPIFLASH::Reset(void){
  CS_Active();
  uint8_t tx_buf[2] = {0};
  tx_buf[0] = CMD_ENABLE_RESET;
  tx_buf[1] = CMD_RESET;
  _spi -> transmit((uint32_t)tx_buf, 2);
  CS_Idle();
}

void SPIFLASH::WaitWriteEnd(void) {
#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

	uint32_t dt = CMD_READ_STATUS_1;
	_spi -> transmit((uint32_t)&dt, 1);
    do {
		_spi -> receive((uint32_t)&flash_properties.SR1, 1);
		delay_ms(1);
    } while ((flash_properties.SR1 & STATUS_REG_BUSY));

#ifdef SPIFLASH_CSPIN_EN
	 CS_Idle();
#endif
}
void SPIFLASH::SetAddress(uint32_t Address){
	uint8_t dt = 0U;
	if(flash_properties.Code >= 256){
		dt = (uint8_t)((Address & 0xFF000000) >> 24);
		_spi -> transmit((uint32_t)&dt, 1);
	}
	dt = (uint8_t)((Address & 0xFF0000) >> 16);
	_spi -> transmit((uint32_t)&dt, 1);
	dt = (uint8_t)((Address & 0x00FF00) >> 8);
	_spi -> transmit((uint32_t)&dt, 1);
	dt = (uint8_t)((Address & 0x0000FF)&0xFF);
	_spi -> transmit((uint32_t)&dt, 1);
}

uint32_t SPIFLASH::ReadID(void) {
    uint32_t ID = 0;
    uint8_t RxBuf[3] = {0};
//    uint8_t dummy[3] = {0};

#ifdef SPIFLASH_CSPIN_EN
    CS_Idle();
#endif
    delay_ms(100);
#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif
    uint8_t dt = CMD_GET_JEDEC_ID;
    _spi -> transmit((uint32_t)&dt, 1);
    _spi -> receive((uint32_t)RxBuf, 3);

#ifdef SPIFLASH_CSPIN_EN
    CS_Idle();
#endif
    ID = (uint32_t)(RxBuf[0]<<16) | (uint32_t)(RxBuf[1]<<8) | (uint32_t)RxBuf[2];
    return ID;
}

uint32_t SPIFLASH::Init(spi_t Spi){
	_spi = Spi;
#ifdef SPIFLASH_CSPIN_EN
	gpio_set_mode(_csport, _cspin, GPIO_OUTPUT_PUSHPULL);
#endif
	delay_ms(100);
	Reset();
	delay_ms(100);
	uint32_t ID = ReadID();
	switch(ID & 0x000000FF){
		case 0x19:	// 	w25q256
			flash_properties.Code = 256;
			flash_properties.BlockCount = 512;
		break;
		case 0x18:	// 	w25q128
			flash_properties.Code = 128;
			flash_properties.BlockCount = 256;
		break;
		case 0x17:	//	w25q64
			flash_properties.Code = 64;
			flash_properties.BlockCount = 128;
		break;
		case 0x16:	//	w25q32
			flash_properties.Code = 32;
			flash_properties.BlockCount = 64;
		break;
		case 0x15:	//	w25q16
			flash_properties.Code = 16;
			flash_properties.BlockCount = 32;
		break;
		case 0x14:	//	w25q80
			flash_properties.Code = 80;
			flash_properties.BlockCount = 16;
		break;
		default:
			flash_properties.Code = 0;
		return 0;
	}
	flash_properties.PageSize = 256;
	flash_properties.SectorSize = 4096;
	flash_properties.SectorCount = flash_properties.BlockCount*16;
	flash_properties.PageCount = (flash_properties.SectorCount*flash_properties.SectorSize)/flash_properties.PageSize;
	flash_properties.BlockSize = flash_properties.SectorSize*16;
	flash_properties.Capacity = (flash_properties.SectorCount*flash_properties.SectorSize)/1024;
	ReadStatusRegister(1);
	ReadStatusRegister(2);
	ReadStatusRegister(3);
	return ID;
}

void SPIFLASH::EraseSector(uint32_t Sector_Address){
	WaitWriteEnd();
    WriteEnable();

#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

    uint8_t dt = CMD_SECTOR_ERASE;
    _spi -> transmit((uint32_t)&dt, 1);
	SetAddress(Sector_Address);

#ifdef SPIFLASH_CSPIN_EN
    CS_Idle();
#endif

    WaitWriteEnd();
}
void SPIFLASH::EraseBlock(uint32_t Block_Address) {
	WaitWriteEnd();
    WriteEnable();

#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

    uint8_t dt = CMD_BLOCK_ERASE;
    _spi -> transmit((uint32_t)&dt, 1);
    SetAddress(Block_Address);

#ifdef SPIFLASH_CSPIN_EN
    CS_Idle();
#endif

	WaitWriteEnd();
}
void SPIFLASH::EraseChip(void){
	WriteEnable();

#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

    uint8_t dt = CMD_CHIP_ERASE;
    _spi -> transmit((uint32_t)&dt, 1);

#ifdef SPIFLASH_CSPIN_EN
    CS_Idle();
#endif

	WaitWriteEnd();
}

void SPIFLASH::ReadBytes(uint32_t ReadAddress, uint8_t *Data, uint16_t NumByteRead){
#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

    uint8_t dt = CMD_FAST_READ;
    _spi -> transmit((uint32_t)&dt, 1);
	SetAddress(ReadAddress);
	dt = 0;
	_spi -> transmit((uint32_t)&dt, 1);

	if(_spi -> _rxdma != NULL && _spi -> _txdma != NULL){
		uint8_t *TxDummyBuf = (uint8_t *)malloc(NumByteRead * sizeof(uint8_t));
		memset(TxDummyBuf, 0x00, NumByteRead);
		_spi -> transmit_receive_start_dma((uint32_t)TxDummyBuf, (uint32_t)Data, (uint16_t)NumByteRead);
		while(!dma_flash_rx_flag);
		_spi -> transmit_receive_stop_dma();
		dma_flash_tx_flag = 0;
		dma_flash_rx_flag = 0;
		free(TxDummyBuf);
	}
	else _spi -> receive((uint32_t)Data, NumByteRead);

#ifdef SPIFLASH_CSPIN_EN
    CS_Idle();
#endif
}

void SPIFLASH::WriteBytes(uint32_t WriteAddress, uint8_t *Data, uint16_t NumByteWrite) {
    WaitWriteEnd();
    WriteEnable();

#ifdef SPIFLASH_CSPIN_EN
    CS_Active();
#endif

    uint8_t dt = CMD_PAGE_PROGRAMM;
    _spi -> transmit((uint32_t)&dt, 1);
    SetAddress(WriteAddress);

    if(_spi -> _txdma == NULL)
    _spi -> transmit((uint32_t)Data, NumByteWrite);
    else{
		_spi -> transmit_start_dma((uint32_t)Data, (uint16_t)NumByteWrite);
		while(!dma_flash_tx_flag);
		_spi -> transmit_stop_dma();
		dma_flash_tx_flag = 0;
    }
#ifdef SPIFLASH_CSPIN_EN
    CS_Idle();
#endif

	WriteDisable();
    WaitWriteEnd();
}

#endif /* ENABLE_COMPONENT_SPIFLASH */

