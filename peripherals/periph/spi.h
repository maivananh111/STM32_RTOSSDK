/*
 * spi.h
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */

#ifndef PERIPH_SPI_H_
#define PERIPH_SPI_H_

#include "peripheral_config.h"
#if ENABLE_SPI

#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */

#include "sdkconfig.h"
#include st_header
#include "stdint.h"
#include "system/ret_err.h"
#if ENABLE_DMA
#include "dma.h"
#endif /* ENABLE_DMA */


typedef enum{
	SPI_FULLDUPLEX_MASTER = 0x01U,
	SPI_HALFDUPLEX_MASTER = 0x02U,

	SPI_FULLDUPLEX_SLAVE  = 0x04U,
	SPI_HALFDUPLEX_SLAVE  = 0x08U,
} spi_mode_t;

typedef enum{
	SPI_NORMAL_CONTROL        = 0U,
	SPI_INTERRUPT_CONTROL     = 0x01U,
	SPI_DMA_CONTROL       	  = 0x02U,
	SPI_INTERRUPT_DMA_CONTROL = 0x04U,
} spi_periph_control_t;

typedef enum{
	SPI_RECEIVE_INTERRUPT,
	SPI_TRANSMIT_INTERRUPT,
	SPI_TRANSMIT_RECEIVE_INTERRUPT,
} spi_interruptoption_t;

typedef enum{
	SPI_DATASIZE_8BIT = 0U,
	SPI_DATASIZE_16BIT,
} spi_datasize_t;

typedef enum{
	SPI_BITORDERING_MSB = 0U,
	SPI_BITORDERING_LSB,
} spi_bitodering_t;

typedef enum{
	SPI_CLOCKDIVISION_2 = 0UL,
	SPI_CLOCKDIVISION_4,
	SPI_CLOCKDIVISION_8,
	SPI_CLOCKDIVISION_16,
	SPI_CLOCKDIVISION_32,
	SPI_CLOCKDIVISION_64,
	SPI_CLOCKDIVISION_128,
	SPI_CLOCKDIVISION_256,
} spi_clockdivision_t;

typedef enum{
	SPI_CLOCK_CPOL0_CPHA0 = 0U,
	SPI_CLOCK_CPOL0_CPHA1,
	SPI_CLOCK_CPOL1_CPHA0,
	SPI_CLOCK_CPOL1_CPHA1,
} spi_clocksample_t;

typedef enum {
	SPI_SOFTWARE_NSS,
	SPI_HARDWARE_NSS,
} spi_nss_t;

#if defined(STM32F1)
typedef enum {
	SPI_NSS_INPUT,
	SPI_NSS_OUTPUT,
} spi_nss_direction_t;
#endif /* STM32F1 */

typedef enum{
	SPI_EVENT_NOEVENT,
	SPI_EVENT_TRANSMIT_COMPLETE,
	SPI_EVENT_RECEIVE_COMPLETE,
	SPI_EVENT_ERROR,
} spi_event_t;

typedef struct {
	spi_mode_t 			  mode              = SPI_FULLDUPLEX_MASTER;
	spi_periph_control_t  control           = SPI_NORMAL_CONTROL;
	spi_interruptoption_t interruptoption   = SPI_RECEIVE_INTERRUPT;
	spi_datasize_t 		  datasize          = SPI_DATASIZE_8BIT;
	spi_bitodering_t 	  bitordering  	    = SPI_BITORDERING_MSB;
	spi_clockdivision_t   clockdivision     = SPI_CLOCKDIVISION_4;
	spi_clocksample_t 	  clocksample 	    = SPI_CLOCK_CPOL0_CPHA0;
	uint32_t 			  interruptpriority = 0;
	spi_nss_t             nss 				= SPI_SOFTWARE_NSS;
#if defined(STM32F1)
	spi_nss_direction_t   nss_dir           = SPI_NSS_INPUT;
#endif /* STM32F1 */
	GPIO_TypeDef 		  *clkport;
	uint16_t 			  clkpin;
	GPIO_TypeDef 		  *misoport;
	uint16_t 			  misopin;
	GPIO_TypeDef 		  *mosiport;
	uint16_t 			  mosipin;
	GPIO_TypeDef 		  *nssport;
	uint16_t 			  nsspin;
#if ENABLE_DMA
	dma_t 				  txdma = NULL;
	dma_t 				  rxdma = NULL;
#endif /* ENABLE_DMA */
} spi_config_t;

class spi {
	public:
		spi(SPI_TypeDef *spi);
		stm_ret_t init(spi_config_t *conf);
		stm_ret_t deinit(void);

		stm_ret_t register_event_handler(void (*function_ptr)(spi_event_t event, void *param), void *param = NULL);
		spi_config_t *get_config(void);

		stm_ret_t transmit(uint32_t data, uint32_t size);
		stm_ret_t receive(uint32_t data, uint32_t size);
		stm_ret_t transmit_receive(uint32_t txdata, uint32_t rxdata, uint32_t size);


		stm_ret_t transmit_start_it(uint32_t data, uint32_t size);
		stm_ret_t receive_start_it(uint32_t data, uint32_t size);
		stm_ret_t transmit_receive_start_it(uint32_t txdata, uint32_t rxdata, uint32_t size);

		stm_ret_t transmit_stop_it(void);
		stm_ret_t receive_stop_it(void);
		stm_ret_t transmit_receive_stop_it(void);

#if ENABLE_DMA
		stm_ret_t transmit_start_dma(uint32_t data, uint32_t size);
		stm_ret_t receive_start_dma(uint32_t data, uint32_t size);
		stm_ret_t transmit_receive_start_dma(uint32_t txdata, uint32_t rxdata, uint32_t size);

		stm_ret_t transmit_stop_dma(void);
		stm_ret_t receive_stop_dma(void);
		stm_ret_t transmit_receive_stop_dma(void);


		dma_t _txdma = NULL, _rxdma = NULL;
#endif /* ENABLE_DMA */
		spi_config_t *_conf = NULL;

		SPI_TypeDef *_spi;
		void *parameter = NULL;
		void (*handler_callback)(spi_event_t event, void *param) = NULL;

		uint32_t txbuf = 0U;
		uint32_t rxbuf = 0U;
		uint32_t txcount = 0U;
		uint32_t txlen = 0U;
		uint32_t rxcount = 0U;
		uint32_t rxlen = 0U;

	private:

		IRQn_Type IRQn;

};

typedef spi* spi_t;

#if defined(SPI1)
extern spi_t spi1;
void SPI1_IRQHandler(void);
#endif /* defined(SPI1) */
#if defined(SPI2)
extern spi_t spi2;
void SPI2_IRQHandler(void);
#endif /* defined(SPI2) */
#if defined(SPI3)
extern spi_t spi3;
void SPI3_IRQHandler(void);
#endif /* defined(SPI3) */
#if defined(SPI4)
extern spi_t spi4;
void SPI4_IRQHandler(void);
#endif /* defined(SPI4) */
#if defined(SPI5)
extern spi_t spi5;
void SPI5_IRQHandler(void);
#endif /* defined(SPI5) */
#if defined(SPI6)
extern spi_t spi6;
void SPI6_IRQHandler(void);
#endif /* defined(SPI6) */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ENABLE_SPI */

#endif /* PERIPH_SPI_H_ */
