/*
 * dma.h
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */

#ifndef PERIPH_DMA_H_
#define PERIPH_DMA_H_


#include "peripheral_config.h"

#if ENABLE_DMA

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "system/ret_err.h"


typedef enum{
#if defined(STM32F4)
	DMA_Channel0 = (0U),
#endif /* STM32F1 */
	DMA_Channel1,
	DMA_Channel2,
	DMA_Channel3,
	DMA_Channel4,
	DMA_Channel5,
	DMA_Channel6,
	DMA_Channel7,
}dma_channel_t;

typedef enum{
	DMA_PERIH_TO_MEM = (0U),
	DMA_MEM_TO_PERIPH,
	DMA_MEM_TO_MEM,
}dma_direction_t;

typedef enum{
	DMA_MODE_NORMAL = (0U),
	DMA_MODE_CIRCULAR,
} dma_mode_t;

typedef enum{
	DMA_DATASIZE_8BIT,
	DMA_DATASIZE_16BIT,
	DMA_DATASIZE_32BIT,
} dma_datasize_t;

#if defined(STM32F4)
typedef enum{
	DMA_NOFIFO = (0U),
	DMA_FIF0 = DMA_SxFCR_DMDIS,
}dma_fifo_t;

typedef enum{
	DMA_SINGLE_TRANFER = (0U),
	DMA_BURST_4INCREMENTAL,
	DMA_BURST_8INCREMENTAL,
	DMA_BURST_16INCREMENTAL,
} dma_burst_t;
#endif /* STM32F4 */

typedef enum{
	DMA_CHANNEL_PRIORITY_LOW = (0U),
	DMA_CHANNEL_PRIORITY_MEDIUM,
	DMA_CHANNEL_PRIORITY_HIGH,
	DMA_CHANNEL_PRIORITY_VERYHIGH,
} dma_channelpriority_t;

#if defined(STM32F1)
typedef enum{
	DMA_TRANSFER_COMPLETE_INTERRUPT = DMA_CCR_TCIE,
	DMA_HALF_TRANSFER_INTERRUPT     = DMA_CCR_HTIE,
	DMA_TRANSFER_ERROR_INTERRUPT    = DMA_CCR_TEIE,
} dma_interruptoption_t;
/** -------------------------------------------------------------------------------------------------- **/
#elif defined(STM32F4)
typedef enum{
	DMA_TRANSFER_COMPLETE_INTERRUPT = DMA_SxCR_TCIE,
	DMA_HALF_TRANSFER_INTERRUPT = DMA_SxCR_HTIE,
	DMA_TRANSFER_ERROR_INTERRUPT = DMA_SxCR_TEIE,
	DMA_DIRECTMODE_ERROR_INTERRUPT = DMA_SxCR_DMEIE,
} dma_interruptoption_t;
#endif /* STM32F4 */

typedef enum{
	DMA_EVENT_NOEVENT,
	DMA_EVENT_TRANFER_COMPLETE,
	DMA_EVENT_HALF_TRANFER,
	DMA_EVENT_TRANFER_ERROR,
} dma_event_t;

typedef struct {
#if defined(STM32F4)
	DMA_Stream_TypeDef 	  *stream;
#endif /* STM32F4 */
	dma_channel_t 		  channel;
	dma_direction_t 	  direction = DMA_MEM_TO_PERIPH;
	dma_mode_t 	 		  mode = DMA_MODE_NORMAL;
	dma_datasize_t 		  datasize = DMA_DATASIZE_8BIT;
#if defined(STM32F4)
	dma_fifo_t 		      fifo = DMA_NOFIFO;
	dma_burst_t 		  burst = DMA_BURST_4INCREMENTAL;
#endif /* STM32F4 */
	uint32_t 			  interruptoption = DMA_TRANSFER_COMPLETE_INTERRUPT | DMA_HALF_TRANSFER_INTERRUPT | DMA_TRANSFER_ERROR_INTERRUPT;
	dma_channelpriority_t channelpriority = DMA_CHANNEL_PRIORITY_HIGH;
	uint32_t 			  interruptpriority = 0;
} dma_config_t;


class dma {
	public:
		dma(DMA_TypeDef *dma);
		stm_ret_t init(dma_config_t *conf);

		void register_event_handler(void (*Event_Callback)(dma_event_t event, void *Parameter), void *Parameter);

		stm_ret_t start(uint32_t Src_Address, uint32_t Dest_Address, uint32_t Number_Data);
		stm_ret_t stop(void);

		uint16_t get_counter(void);

		stm_ret_t poll_for_tranfer(dma_interruptoption_t PollLevel, uint32_t TimeOut);

		dma_config_t *get_config(void);

		void (*Event_Callback)(dma_event_t event, void *Parameter);
		void *Parameter;

	private:
		DMA_TypeDef *_dma;

#if defined(STM32F1)
		DMA_Channel_TypeDef *_dma_channel;
#endif /* STM32F1 */
		dma_config_t *_conf;
		stm_err_t _state = STM_READY;
		IRQn_Type _IRQn = (IRQn_Type)0;

#if defined(STM32F4)
		__IO uint32_t _Intr_Index = 0U;
		uint32_t Stream = 0U;
		uint8_t DMA_Reg_Level = 0U; // 0 is LOW register.
		volatile uint32_t *ICFR = 0x00000000U;
		volatile uint32_t *ISR  = 0x00000000U;
#endif /* STM32F4 */

		void ClearIFCR(__IO uint32_t Value);
		void ClearAllIntrFlag(void);
		__IO uint32_t GetISR(void);
};

typedef dma* dma_t;

#if defined(STM32F1)
void DMA_IRQ_Handler(DMA_TypeDef *pdma, DMA_Channel_TypeDef *pchannel, dma *dmaptr);
/** -------------------------------------------------------------------------------------------------- **/
#elif defined(STM32F4)
void DMA_IRQ_Handler(DMA_TypeDef *pdma, DMA_Stream_TypeDef *stream, dma *dmaptr);
#endif /* STM32F4 */

/**
 *  DMA1 channel class.
 */
/* DMA1 IRQ HANDLER */
#if defined(DMA1_Channel1)
extern dma_t dma1_channel1;
void DMA1_Channel1_IRQHandler(void);
#endif /* defined(DMA1_Channel1) */
#if defined(DMA1_Channel2)
extern dma_t dma1_channel2;
void DMA1_Channel2_IRQHandler(void);
#endif /* defined(DMA1_Channel2) */
#if defined(DMA1_Channel3)
extern dma_t dma1_channel3;
void DMA1_Channel3_IRQHandler(void);
#endif /* defined(DMA1_Channel3) */
#if defined(DMA1_Channel4)
extern dma_t dma1_channel4;
void DMA1_Channel4_IRQHandler(void);
#endif /* defined(DMA1_Channel4) */
#if defined(DMA1_Channel5)
extern dma_t dma1_channel5;
void DMA1_Channel5_IRQHandler(void);
#endif /* defined(DMA1_Channel5) */
#if defined(DMA1_Channel6)
extern dma_t dma1_channel6;
void DMA1_Channel6_IRQHandler(void);
#endif /* defined(DMA1_Channel6) */
#if defined(DMA1_Channel7)
extern dma_t dma1_channel7;
void DMA1_Channel7_IRQHandler(void);
#endif /* defined(DMA1_Channel7) */

/**
 *  DMA2 channel class.
 */
#if defined(DMA2_Channel1)
extern dma_t dma2_channel1;
void DMA2_Channel1_IRQHandler(void);
#endif /* defined(DMA2_Channel1) */
#if defined(DMA2_Channel2)
extern dma_t dma2_channel2;
void DMA2_Channel2_IRQHandler(void);
#endif /* defined(DMA2_Channel2) */
#if defined(DMA2_Channel3)
extern dma_t dma2_channel3;
void DMA2_Channel3_IRQHandler(void);
#endif /* defined(DMA2_Channel3) */
#if defined(DMA2_Channel4)
extern dma_t dma2_channel4;
void DMA2_Channel4_IRQHandler(void);
#endif /* defined(DMA2_Channel4) */
#if defined(DMA2_Channel5)
extern dma_t dma2_channel5;
void DMA2_Channel5_IRQHandler(void);
#endif /* defined(DMA2_Channel5) */
#if defined(DMA2_Channel6)
extern dma_t dma2_channel6;
void DMA2_Channel6_IRQHandler(void);
#endif /* defined(DMA2_Channel6) */
#if defined(DMA2_Channel7)
extern dma_t dma2_channel7;
void DMA2_Channel7_IRQHandler(void);
#endif /* defined(DMA2_Channel7) */


/**
 *  DMA1 Stream class.
 */
/* DMA1 IRQ HANDLER */
#if defined(DMA1_Stream0)
extern dma_t dma1_stream0;
void DMA1_Stream0_IRQHandler(void);
#endif /* defined(DMA1_Stream0) */
#if defined(DMA1_Stream1)
extern dma_t dma1_stream1;
void DMA1_Stream1_IRQHandler(void);
#endif /* defined(DMA1_Stream1) */
#if defined(DMA1_Stream2)
extern dma_t dma1_stream2;
void DMA1_Stream2_IRQHandler(void);
#endif /* defined(DMA1_Stream2) */
#if defined(DMA1_Stream3)
extern dma_t dma1_stream3;
void DMA1_Stream3_IRQHandler(void);
#endif /* defined(DMA1_Stream3) */
#if defined(DMA1_Stream4)
extern dma_t dma1_stream4;
void DMA1_Stream4_IRQHandler(void);
#endif /* defined(DMA1_Stream4) */
#if defined(DMA1_Stream5)
extern dma_t dma1_stream5;
void DMA1_Stream5_IRQHandler(void);
#endif /* defined(DMA1_Stream5) */
#if defined(DMA1_Stream6)
extern dma_t dma1_stream6;
void DMA1_Stream6_IRQHandler(void);
#endif /* defined(DMA1_Stream6) */
#if defined(DMA1_Stream7)
extern dma_t dma1_stream7;
void DMA1_Stream7_IRQHandler(void);
#endif /* defined(DMA1_Stream7) */

/**
 *  DMA2 Stream class.
 */
#if defined(DMA2_Stream0)
extern dma_t dma2_stream0;
void DMA2_Stream0_IRQHandler(void);
#endif /* defined(DMA2_Stream0) */
#if defined(DMA2_Stream1)
extern dma_t dma2_stream1;
void DMA2_Stream1_IRQHandler(void);
#endif /* defined(DMA2_Stream1) */
#if defined(DMA2_Stream2)
extern dma_t dma2_stream2;
void DMA2_Stream2_IRQHandler(void);
#endif /* defined(DMA2_Stream2) */
#if defined(DMA2_Stream3)
extern dma_t dma2_stream3;
void DMA2_Stream3_IRQHandler(void);
#endif /* defined(DMA2_Stream3) */
#if defined(DMA2_Stream4)
extern dma_t dma2_stream4;
void DMA2_Stream4_IRQHandler(void);
#endif /* defined(DMA2_Stream4) */
#if defined(DMA2_Stream5)
extern dma_t dma2_stream5;
void DMA2_Stream5_IRQHandler(void);
#endif /* defined(DMA2_Stream5) */
#if defined(DMA2_Stream6)
extern dma_t dma2_stream6;
void DMA2_Stream6_IRQHandler(void);
#endif /* defined(DMA2_Stream6) */
#if defined(DMA2_Stream7)
extern dma_t dma2_stream7;
void DMA2_Stream7_IRQHandler(void);
#endif /* defined(DMA2_Stream7) */

#ifdef __cplusplus
}
#endif

#endif /* ENABLE_DMA */

#endif /* PERIPH_DMA_H_ */
