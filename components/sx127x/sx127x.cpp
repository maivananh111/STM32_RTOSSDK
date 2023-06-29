/*
 * sx127x.cpp
 *
 *  Created on: Mar 26, 2023
 *      Author: anh
 */

#include "component_config.h"
#if ENABLE_COMPONENT_SX127X_LORA

#include "sx127x.h"
#include "periph/gpio.h"
#include "periph/exti.h"
#include "periph/systick.h"



sx127x::sx127x(GPIO_TypeDef *csport, uint16_t cspin, GPIO_TypeDef *rstport, uint16_t rstpin, GPIO_TypeDef *itport, uint16_t itpin){
	_csport = csport;
	_rstport = rstport;
	_itport = itport;

	_cs = cspin;
	_rst = rstpin;
	_it = itpin;
}

bool sx127x::init(spi_t spi, long frequency, uint8_t power, uint32_t interruptpriority){
	_spi = spi;

	gpio_port_clock_enable(_csport);
	gpio_port_clock_enable(_rstport);
	gpio_port_clock_enable(_itport);

	gpio_set_mode(_csport, _cs, GPIO_OUTPUT_PUSHPULL);
	gpio_set(_csport, _cs);

	gpio_set_mode(_rstport, _rst, GPIO_OUTPUT_PUSHPULL);
	gpio_reset(_rstport, _rst);
    delay_ms(50);
    gpio_set(_rstport, _rst);
    delay_ms(50);

    if(_it >= 0 && _itport != NULL){
		exti_init(_itport, _it, EXTI_RISING_EDGE, interruptpriority);
		gpio_set_pulldown(_itport, _it);
    }

	uint8_t version = readRegister(REG_VERSION);
	if(version != 0x12) return false;

	sleep();
	setFrequency(frequency);
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
	writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
	writeRegister(REG_MODEM_CONFIG_3, 0x04);
	setTxPower(power);
	enableCrc();
	idle();

	return true;
}

void sx127x::register_event_handler(void (*TxHandler)(void *arg), void (*RxHandler)(void *arg, uint8_t len)){
	TxDoneHandler = TxHandler;
	RxDoneHandler = RxHandler;
}

bool sx127x::beginPacket(bool implicitHeader){
	if (isTransmitting()) return false;

	idle();

	if (implicitHeader) implicitHeaderMode();
	else explicitHeaderMode();

	writeRegister(REG_FIFO_ADDR_PTR, 0);
	writeRegister(REG_PAYLOAD_LENGTH, 0);

	return true;
}

bool sx127x::endPacket(bool async){
	if(async && (TxDoneHandler)) writeRegister(REG_DIO_MAPPING_1, 0x40);

	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

	if(!async){
		while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);

		writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}

	return true;
}

bool sx127x::isTransmitting(void){
	if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) return true;

	if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

	return false;
}

uint8_t sx127x::parsePacket(uint8_t size){
	uint8_t packetLength = 0;
	uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);

	if(size > 0) {
		implicitHeaderMode();
		writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	}
	else
		explicitHeaderMode();

	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0){
		_packetIndex = 0;

		if (_implicitHeaderMode) packetLength = readRegister(REG_PAYLOAD_LENGTH);
		else packetLength = readRegister(REG_RX_NB_BYTES);

		writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

		idle();
	}
	else if(readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
		writeRegister(REG_FIFO_ADDR_PTR, 0);

		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}

	return packetLength;
}

int sx127x::packetRssi(void){
	return (int)(readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float sx127x::packetSnr(void){
	return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long sx127x::packetFrequencyError(void){
	int32_t freqError = 0;
	freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 0b111);
	freqError <<= 8L;
	freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
	freqError <<= 8L;
	freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

	if (readRegister(REG_FREQ_ERROR_MSB) & 0b1000) freqError -= 524288;

	const float fXtal = 32E6;
	const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

	return static_cast<long>(fError);
}

int16_t sx127x::rssi(void){
    return (readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t sx127x::transmit(uint8_t byte){
  return transmit(&byte, sizeof(byte));
}

size_t sx127x::transmit(const uint8_t *buffer, size_t size){
	int currentLength = readRegister(REG_PAYLOAD_LENGTH);

	if ((currentLength + size) > MAX_PKT_LENGTH) size = MAX_PKT_LENGTH - currentLength;

	for (size_t i = 0; i < size; i++) writeRegister(REG_FIFO, buffer[i]);

	writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

	return size;
}

uint8_t sx127x::available(void){
	return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

uint8_t sx127x::receive(char *buffer){
	uint8_t i = 0;
	while(available()){
		_packetIndex++;

		buffer[i++] = readRegister(REG_FIFO);
	}
	return _packetIndex;
}

uint8_t sx127x::peek(void){
	if(!available()) return -1;

	int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

	uint8_t b = readRegister(REG_FIFO);

	writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

	return b;
}

void sx127x::receive_it(uint8_t size){
	if(RxDoneHandler)writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

	if (size > 0) {
		implicitHeaderMode();

		writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	}
	else {
		explicitHeaderMode();
	}

	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void sx127x::idle(void){
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sx127x::sleep(void){
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void sx127x::setTxPower(uint8_t level, uint8_t outputPin){
	if(PA_OUTPUT_RFO_PIN == outputPin) {
		if(level < 0) level = 0;
		else if(level > 14) level = 14;

		writeRegister(REG_PA_CONFIG, 0x70 | level);
	}
	else{
		if(level > 17){
			if (level > 20) level = 20;
			level -= 3;

			writeRegister(REG_PA_DAC, 0x87);
			setOCP(140);
		}
		else {
			if (level < 2) level = 2;
			writeRegister(REG_PA_DAC, 0x84);
			setOCP(100);
		}

		writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
	}
}

void sx127x::setFrequency(long frequency){
	_frequency = frequency;

	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

uint8_t sx127x::getSpreadingFactor(void){
	return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void sx127x::setSpreadingFactor(uint8_t sf){
	if (sf < 6) sf = 6;
	else if (sf > 12) sf = 12;


	if (sf == 6) {
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
	}
	else {
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
	setLdoFlag();
}

long sx127x::getSignalBandwidth(void){
	uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

	switch (bw) {
		case 0: return 7.8E3;
		case 1: return 10.4E3;
		case 2: return 15.6E3;
		case 3: return 20.8E3;
		case 4: return 31.25E3;
		case 5: return 41.7E3;
		case 6: return 62.5E3;
		case 7: return 125E3;
		case 8: return 250E3;
		case 9: return 500E3;
	}

	return -1;
}

void sx127x::setSignalBandwidth(long sbw){
	int bw;

	if (sbw <= 7.8E3)        bw = 0;
	else if (sbw <= 10.4E3)  bw = 1;
	else if (sbw <= 15.6E3)  bw = 2;
	else if (sbw <= 20.8E3)  bw = 3;
	else if (sbw <= 31.25E3) bw = 4;
	else if (sbw <= 41.7E3)  bw = 5;
	else if (sbw <= 62.5E3)  bw = 6;
	else if (sbw <= 125E3)   bw = 7;
	else if (sbw <= 250E3)   bw = 8;
	else/*if (sbw <= 250E3)*/bw = 9;

	writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
	setLdoFlag();
}

void sx127x::setLdoFlag(void){
	long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

	bool ldoOn = symbolDuration > 16;

	uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
	if(ldoOn) config3 |= (1<<3);
	else config3 &=~ (1<<3);
	writeRegister(REG_MODEM_CONFIG_3, config3);
}

void sx127x::setCodingRate4(uint8_t denominator){
	if (denominator < 5) denominator = 5;
	else if (denominator > 8) denominator = 8;

	uint8_t cr = denominator - 4;

	writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void sx127x::setPreambleLength(long length){
	writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
	writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void sx127x::setSyncWord(uint8_t sw){
	writeRegister(REG_SYNC_WORD, sw);
}

void sx127x::enableCrc(void){
	writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void sx127x::disableCrc(void){
	writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void sx127x::enableInvertIQ(void){
	writeRegister(REG_INVERTIQ,  0x66);
	writeRegister(REG_INVERTIQ2, 0x19);
}

void sx127x::disableInvertIQ(void){
	writeRegister(REG_INVERTIQ,  0x27);
	writeRegister(REG_INVERTIQ2, 0x1d);
}

void sx127x::setOCP(uint8_t mA){
	uint8_t ocpTrim = 27;

	if (mA <= 120) ocpTrim = (mA - 45) / 5;
	else if (mA <=240) ocpTrim = (mA + 30) / 10;

	writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void sx127x::setGain(uint8_t gain){
	if (gain > 6) gain = 6;

	idle();

	if (gain == 0) writeRegister(REG_MODEM_CONFIG_3, 0x04);
	else {
		writeRegister(REG_MODEM_CONFIG_3, 0x00);

		writeRegister(REG_LNA, 0x03);

		writeRegister(REG_LNA, readRegister(REG_LNA) | (gain << 5));
	}
}


void sx127x::explicitHeaderMode(void){
	_implicitHeaderMode = 0;

	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void sx127x::implicitHeaderMode(void){
	_implicitHeaderMode = 1;

	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void sx127x::IRQHandler(void){
	uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);

	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
			_packetIndex = 0;

			uint8_t packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

			writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

			if (RxDoneHandler) {
				RxDoneHandler(this, packetLength);
			}
		}
		else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
			if (TxDoneHandler) {
				TxDoneHandler(this);
			}
		}
	}
}

uint8_t sx127x::readRegister(uint8_t address){
	return singleTransfer(address & 0x7f, 0x00);
}

void sx127x::writeRegister(uint8_t address, uint8_t value){
	singleTransfer(address | 0x80, value);
}

uint8_t sx127x::singleTransfer(uint8_t address, uint8_t value){
  uint8_t response, txdt;

  gpio_reset(_csport, _cs);

  txdt = address;
  _spi -> transmit((uint32_t)(&txdt), 1);
  txdt = value;
  _spi -> transmit_receive((uint32_t)(&txdt), (uint32_t)(&response), 1);

  gpio_set(_csport, _cs);

  return response;
}

#endif /* ENABLE_COMPONENT_SX127X_LORA */
