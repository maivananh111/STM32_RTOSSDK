/*
 * usart.h
 *
 *  Created on: 29 thg 7, 2022
 *      Author: A315-56
 */

#ifndef PERIPH_USART_H_
#define PERIPH_USART_H_


#include "peripheral_config.h"
#if ENABLE_USART

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include st_header
#include "stdio.h"
#if ENABLE_DMA
#include "dma.h"
#endif
#include "system/ret_err.h"


typedef enum{
	USART_NORMAL_CONTROL 	    = 0x00U,
	USART_INTERRUPT_CONTROL     = 0x01U,
	USART_DMA_CONTROl           = 0x02U,
	USART_INTERRUPT_DMA_CONTROL = 0x04U,
} usart_periph_control_t;

typedef enum{
	USART_RECEIVE_INTERRUPT          = 0x01U,
	USART_TRANSMIT_INTERRUPT         = 0x02U,
	USART_TRANSMIT_RECEIVE_INTERRUPT = 0x04U,
} usart_interruptoption_t;

typedef enum{
	USART_EVENT_NOEVENT,
	USART_EVENT_TRANSMIT_COMPLETE,
	USART_EVENT_RECEIVE_COMPLETE,
	USART_EVENT_BUFFER_OVERFLOW,
	USART_EVENT_IDLE_STATE,
	USART_EVENT_RECEIVE_ENDCHAR,
	USART_EVENT_ERROR,
} usart_event_t;

typedef enum{
	USART_RECEPTION_NORMAL,
	USART_RECEPTION_TOENDCHAR,
	USART_RECEPTION_TOIDLE,
} usart_reception_t;

typedef struct{
	uint32_t 			     baudrate;
	usart_periph_control_t   control = USART_NORMAL_CONTROL;
	usart_interruptoption_t  interruptoption = USART_RECEIVE_INTERRUPT;
	uint32_t 			     interruptpriority = 0;
	GPIO_TypeDef 		     *txport;
	uint16_t 			     txpin;
	GPIO_TypeDef 		     *rxport;
	uint16_t 			     rxpin;
#if ENABLE_DMA
	dma_t 				     txdma = NULL;
	dma_t 				     rxdma = NULL;
#endif /* ENABLE_DMA */
} usart_config_t;



class usart {
	public:
		usart(USART_TypeDef *usart);
		stm_ret_t init(usart_config_t *conf);
		stm_ret_t deinit(void);

		stm_ret_t register_event_handler(void (*function_ptr)(usart_event_t event, void *param), void *param = NULL);

		stm_ret_t transmit(uint8_t data);
		stm_ret_t transmit(uint8_t *data, uint16_t len);
		stm_ret_t send_string(char *string);

		stm_ret_t receive(uint16_t len);
		stm_ret_t receive(uint8_t *data);
		uint8_t  get_data(void);

		stm_ret_t transmit_start_it(uint8_t *data, uint16_t len);
		stm_ret_t receive_start_it(uint16_t buffer_size);
		stm_ret_t transmit_stop_it(void);
		stm_ret_t receive_stop_it(void);

#if ENABLE_DMA
		stm_ret_t transmit_start_dma(uint8_t *data, uint16_t len);
		stm_ret_t receive_start_dma(uint16_t len);
		stm_ret_t transmit_stop_dma(void);
		stm_ret_t receive_stop_dma(void);
#endif /* ENABLE_DMA */

		stm_ret_t receive_to_idle_start_it(uint16_t buffer_size);
		stm_ret_t receive_to_idle_stop_it(void);
#if ENABLE_DMA
		stm_ret_t receive_to_idle_start_it_dma(uint16_t buffer_size);
		stm_ret_t receive_to_idle_stop_it_dma(void);
#endif /* ENABLE_DMA */
		stm_ret_t receive_to_endchar_start_it(uint16_t buffer_size, char endchar = '\0');
		stm_ret_t receive_to_endchar_stop_it(void);
#if ENABLE_DMA
		stm_ret_t receive_to_endchar_start_dma(uint16_t buffer_size, char endchar = '\0');
		stm_ret_t receive_to_endchar_stop_dma(void);
#endif /* ENABLE_DMA */

		stm_ret_t get_buffer(uint8_t **data);
		uint16_t get_bufferlen(void);

		usart_config_t *get_config(void);

		USART_TypeDef *_usart;
#if ENABLE_DMA
		dma_t _txdma = NULL, _rxdma = NULL;
#endif /* ENABLE_DMA */
		void *parameter = NULL;
		void (*handler_callback)(usart_event_t event, void *param) = NULL;
		uint8_t *rxbuffer = NULL;
		uint16_t rxlen, rxcount;
		char endchar = '\0';
		usart_reception_t reception = USART_RECEPTION_NORMAL;

	private:
		usart_config_t *_conf = NULL;
		IRQn_Type IRQn;

};

typedef usart* usart_t;


#if defined(USART1)
extern usart_t usart1;
void USART1_IRQHandler(void);
#endif
#if defined(USART2)
extern usart_t usart2;
void USART2_IRQHandler(void);
#endif
#if defined(USART3)
extern usart_t usart3;
void USART3_IRQHandler(void);
#endif
#if defined(UART4)
extern usart_t uart4;
void UART4_IRQHandler(void);
#endif
#if defined(UART5)
extern usart_t uart5;
void UART5_IRQHandler(void);
#endif
#if defined(USART6)
extern usart_t usart6;
void USART6_IRQHandler(void);
#endif
#if defined(UART7)
extern usart_t uart7;
void UART7_IRQHandler(void);
#endif
#if defined(UART8)
extern usart_t uart8;
void UART8_IRQHandler(void);
#endif


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_USART */

#endif /* PERIPH_USART_H_ */

