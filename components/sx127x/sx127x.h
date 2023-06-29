/*
 * sx127x.h
 *
 *  Created on: Mar 26, 2023
 *      Author: anh
 */

#ifndef COMPONENTS_SX127X_H_
#define COMPONENTS_SX127X_H_

#include "component_config.h"
#if ENABLE_COMPONENT_SX127X_LORA

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "periph/spi.h"


// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d
// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
// PA config
#define PA_BOOST                 0x80
// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

#define INT_PRIORITY 3

class sx127x{
	public:
		sx127x(GPIO_TypeDef *csport, uint16_t cspin, GPIO_TypeDef *rstport, uint16_t rstpin, GPIO_TypeDef *itport = NULL, uint16_t itpin = -1);

		bool init(spi_t spi, long frequency, uint8_t power, uint32_t interruptpriority);
		void register_event_handler(void (*TxHandler)(void *arg), void (*RxHandler)(void *arg, uint8_t len));
		bool beginPacket(bool implicitHeader = false);
		bool endPacket(bool async = false);
		void Receive(uint8_t size);

		uint8_t parsePacket(uint8_t size = 0);
		int packetRssi(void);
		float packetSnr(void);
		long packetFrequencyError(void);

		int16_t rssi(void);
		size_t transmit(uint8_t byte);
		size_t transmit(const uint8_t *buffer, size_t size);

		uint8_t available(void);
		uint8_t receive(char *buffer);
		uint8_t peek(void);

		void idle(void);
		void sleep(void);

		void setTxPower(uint8_t level, uint8_t outputPin = PA_OUTPUT_PA_BOOST_PIN);
		void setFrequency(long frequency);
		void setSpreadingFactor(uint8_t sf);
		void setSignalBandwidth(long sbw);
		void setCodingRate4(uint8_t denominator);
		void setPreambleLength(long length);
		void setSyncWord(uint8_t sw);
		void enableCrc(void);
		void disableCrc(void);
		void enableInvertIQ(void);
		void disableInvertIQ(void);

		void setOCP(uint8_t mA);
		void setGain(uint8_t gain);

		void IRQHandler(void);


	protected:
		uint8_t readRegister(uint8_t address);
		void writeRegister(uint8_t address, uint8_t value);
		uint8_t singleTransfer(uint8_t address, uint8_t value);

	private:
		void explicitHeaderMode(void);
		void implicitHeaderMode(void);

		bool isTransmitting(void);

		uint8_t getSpreadingFactor();
		long getSignalBandwidth(void);

		void setLdoFlag(void);

		spi_t _spi;

		uint16_t _cs, _rst;
		int16_t _it = -1;
		GPIO_TypeDef *_csport, *_rstport;
		GPIO_TypeDef *_itport = NULL;
		int 		_packetIndex = 0;
		int 		_implicitHeaderMode = 0;
		long		_frequency;
		void        (*TxDoneHandler)(void *arg) = NULL;
		void        (*RxDoneHandler)(void *arg, uint8_t len) = NULL;
};



#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_SX127X_LORA */

#endif /* COMPONENTS_SX127X_H_ */
