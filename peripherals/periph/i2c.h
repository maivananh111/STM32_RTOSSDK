/*
 * i2c.h
 *
 *  Created on: Mar 21, 2023
 *      Author: anh
 */

#ifndef PERIPH_I2C_H_
#define PERIPH_I2C_H_


#include "peripheral_config.h"
#if ENABLE_I2C

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "system/ret_err.h"
#if ENABLE_DMA
#include "dma.h"
#endif /* ENABLE_DMA */


typedef enum{
	I2C_STANDARD_MODE = 0,
	I2C_FAST_MODE,
} i2c_mode_t;

typedef enum{
	I2C_CLOCKDUTY_2 = 0,
	I2C_CLOCKDUTY_16_9,
} i2c_clockduty_t;

typedef enum{
	I2C_ADDRESSSIZE_7BIT = 0,
	I2C_ADDRESSSIZE_10BIT,
} i2c_addresssize_t;

typedef enum{
	I2C_WRITE = 0,
	I2C_READ,
} i2c_action_t;

typedef struct{
	i2c_mode_t        mode = I2C_STANDARD_MODE;
	uint32_t 		  frequency = 100000U;
	i2c_addresssize_t addresssize = I2C_ADDRESSSIZE_7BIT;
	i2c_clockduty_t   clockduty = I2C_CLOCKDUTY_2;

	GPIO_TypeDef 	  *sclport;
	uint16_t 		  sclpin;
	GPIO_TypeDef 	  *sdaport;
	uint16_t 		  sdapin;
	bool              gpio_remap = false;
#if ENABLE_DMA
	dma_t 			  txdma = NULL;
	dma_t 			  rxdma = NULL;
#endif /* ENABLE_DMA */
} i2c_config_t;

class i2c{
	public:
		i2c(I2C_TypeDef *i2c);
		stm_ret_t init(i2c_config_t *conf);

		stm_ret_t check_device(uint16_t slaveaddress, uint8_t trials, uint16_t timeout);

		stm_ret_t send_start(void);
		stm_ret_t send_repeatstart(void);
		stm_ret_t send_slaveaddress(uint16_t slaveaddress, i2c_action_t action);
		stm_ret_t send_start_slaveaddress(uint16_t slaveaddress, i2c_action_t action);
		stm_ret_t send_repeatstart_slaveaddress(uint16_t slaveaddress, i2c_action_t action);

		stm_ret_t send_stop(void);

		stm_ret_t transmit(uint8_t *data, uint16_t size);
		stm_ret_t transmit(uint8_t data);

		stm_ret_t receive(uint8_t *data, uint16_t size);
		stm_ret_t receive(uint8_t *data);

#if ENABLE_DMA
		stm_ret_t transmit_start_dma(uint8_t *data, uint16_t size);
		stm_ret_t receive_start_dma(uint8_t *data, uint16_t size);

		stm_ret_t transmit_stop_dma(void);
		stm_ret_t receive_stop_dma(void);
#endif /* ENABLE_DMA */

		stm_ret_t i2c_write_register(uint16_t slaveaddress, uint16_t registeraddress, uint8_t registeraddress_size, uint8_t *data, uint16_t size);
		stm_ret_t i2c_read_register(uint16_t slaveaddress, uint16_t registeraddress, uint8_t registeraddress_size, uint8_t *data, uint16_t size);

#if ENABLE_DMA
		dma_t _txdma = NULL, _rxdma = NULL;
#endif /* ENABLE_DMA */

	private:
		I2C_TypeDef *_i2c;
		i2c_config_t *_conf;

		void ACKError_Action(void);
		void ClearADDR(void);
		stm_ret_t WaitBusy(void);
};

typedef i2c* i2c_t;

#if defined(I2C1)
extern i2c_t i2c1;
#endif /* defined(I2C1) */
#if defined(I2C2)
extern i2c_t i2c2;
#endif /* defined(I2C2) */
#if defined(I2C3)
extern i2c_t i2c3;
#endif /* defined(I2C3) */
#if defined(I2C4)
extern i2c_t i2c4;
#endif /* defined(I2C4) */
#if defined(I2C5)
extern i2c_t i2c5;
#endif /* defined(I2C5) */
#if defined(I2C6)
extern i2c_t i2c6;
#endif /* defined(I2C6) */

#ifdef __cplusplus
}
#endif

#endif /* ENABLE_I2C */

#endif /* PERIPH_I2C_H_ */
