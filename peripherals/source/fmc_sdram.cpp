/*
 * fmc_sdram.cpp
 *
 *  Created on: Mar 22, 2023
 *      Author: anh
 */

#include "peripheral_config.h"

#if ENABLE_FMC
#if defined(STM32F429xx)

#include "sdkconfig.h"

#include "periph/fmc_sdram.h"
#include "periph/gpio.h"
#include "periph/systick.h"



#define FMC_SDRAM_CMD_NORMAL_MODE               (0x00000000U)
#define FMC_SDRAM_CMD_CLK_ENABLE                (0x00000001U)
#define FMC_SDRAM_CMD_PALL                      (0x00000002U)
#define FMC_SDRAM_CMD_AUTOREFRESH_MODE          (0x00000003U)
#define FMC_SDRAM_CMD_LOAD_MODE                 (0x00000004U)
#define FMC_SDRAM_CMD_SELFREFRESH_MODE          (0x00000005U)
#define FMC_SDRAM_CMD_POWERDOWN_MODE            (0x00000006U)

#define FMC_SDRAM_CMD_TARGET_BANK2              FMC_SDCMR_CTB2
#define FMC_SDRAM_CMD_TARGET_BANK1              FMC_SDCMR_CTB1
#define FMC_SDRAM_CMD_TARGET_BANK1_2            (0x00000018U)

static sdram_config_t *_conf;
void fmc_sdram_hardware_init(void);
void fmc_sdram_init(sdram_config_t *conf){
	_conf = conf;

	fmc_sdram_hardware_init();

	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

	if(_conf -> bank == SDRAM_BANK1){
		/**
		 * Configure control register.
		*/
		__IO uint32_t tmpreg = SDRAM_DEVICE -> SDCR[SDRAM_BANK1];
		tmpreg &=~ 0xFFFFU;

		tmpreg |= _conf -> clock_cycle | _conf -> coladdr_size
				| _conf -> rowaddr_size | _conf -> data_size
				| _conf -> num_banks | _conf -> cas_latency
			    | _conf ->read_delay;
		if(_conf -> write_protection) tmpreg |= FMC_SDCR1_WP;
		if(_conf -> read_burst) tmpreg |= FMC_SDCR1_RBURST;

		SDRAM_DEVICE -> SDCR[SDRAM_BANK1] = tmpreg;

		/**
		 * Configure timing register.
		*/
		tmpreg = SDRAM_DEVICE ->SDTR[SDRAM_BANK1];
		tmpreg &=~ 0xFFFFFFFF;
		tmpreg |= ((_conf -> timing.tMRD-1U) << FMC_SDTR1_TMRD_Pos)
				| ((_conf -> timing.tXSR-1U) << FMC_SDTR1_TXSR_Pos)
				| ((_conf -> timing.tRAS-1U) << FMC_SDTR1_TRAS_Pos)
				| ((_conf -> timing.tRC -1U) << FMC_SDTR1_TRC_Pos)
				| ((_conf -> timing.tDPL-1U) << FMC_SDTR1_TWR_Pos)
				| ((_conf -> timing.tRP -1U) << FMC_SDTR1_TRP_Pos)
				| ((_conf -> timing.tRCD-1U) << FMC_SDTR1_TRCD_Pos);

		SDRAM_DEVICE ->SDTR[SDRAM_BANK1] = tmpreg;
	}
	else{
		/**
		 * Configure control register.
		*/
		/* SDCLK, BURST, RPIPE must be configure in SDCR1 register */
		__IO uint32_t tmpreg = SDRAM_DEVICE -> SDCR[SDRAM_BANK1];
		tmpreg &=~ (FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE);

		tmpreg |= _conf -> clock_cycle | _conf ->read_delay;
		if(_conf -> read_burst) tmpreg |= FMC_SDCR1_RBURST;

		SDRAM_DEVICE -> SDCR[SDRAM_BANK1] = tmpreg;


		/* Configure SDCR2 register */
		tmpreg = SDRAM_DEVICE -> SDCR[SDRAM_BANK2];
		tmpreg &=~ 0xFFFFU;

		tmpreg |= _conf -> coladdr_size
				| _conf -> rowaddr_size | _conf -> data_size
				| _conf -> num_banks | _conf -> cas_latency;
		if(_conf -> write_protection) tmpreg |= FMC_SDCR1_WP;

		SDRAM_DEVICE -> SDCR[SDRAM_BANK2] = tmpreg;

		/**
		 * Configure timing register.
		*/
		tmpreg = SDRAM_DEVICE ->SDTR[SDRAM_BANK1];
		tmpreg &=~ (FMC_SDTR1_TRC | FMC_SDTR1_TRP);
		tmpreg |= ((_conf -> timing.tRC -1U) << FMC_SDTR1_TRC_Pos)
				| ((_conf -> timing.tRP -1U) << FMC_SDTR1_TRP_Pos);

		SDRAM_DEVICE ->SDTR[SDRAM_BANK1] = tmpreg;


		tmpreg = SDRAM_DEVICE ->SDTR[SDRAM_BANK2];
		tmpreg &=~ 0xFFFFFFFF;
		tmpreg |= ((_conf -> timing.tMRD-1U) << FMC_SDTR1_TMRD_Pos)
				| ((_conf -> timing.tXSR-1U) << FMC_SDTR1_TXSR_Pos)
				| ((_conf -> timing.tRAS-1U) << FMC_SDTR1_TRAS_Pos)
				| ((_conf -> timing.tDPL-1U) << FMC_SDTR1_TWR_Pos)
				| ((_conf -> timing.tRCD-1U) << FMC_SDTR1_TRCD_Pos);

		SDRAM_DEVICE ->SDTR[SDRAM_BANK2] = tmpreg;
	}
}

stm_ret_t fmc_sdram_sendcommand(sdram_command_t cmd){
	stm_ret_t ret;

	__IO uint32_t tmpreg = SDRAM_DEVICE -> SDCMR;
	tmpreg &=~ (FMC_SDCMR_MODE | FMC_SDCMR_CTB2 | FMC_SDCMR_CTB1 | FMC_SDCMR_NRFS | FMC_SDCMR_MRD);

	tmpreg |= cmd.mode | cmd.targer | ((cmd.autorefresh_num-1U) << FMC_SDCMR_NRFS_Pos)
		   | (cmd.registermode << FMC_SDCMR_MRD_Pos);

	SDRAM_DEVICE -> SDCMR = tmpreg;

	ret = wait_flag_in_register_timeout(&(SDRAM_DEVICE -> SDSR), FMC_SDSR_BUSY, FLAG_RESET, FMC_SDRAM_TIMEOUT);

	return ret;
}

void fmc_sdram_setrefreshrate(uint32_t refreshrate){
	__IO uint32_t tmpreg = SDRAM_DEVICE -> SDRTR;
	tmpreg &=~ FMC_SDRTR_COUNT;

	tmpreg |= (refreshrate << FMC_SDRTR_COUNT_Pos);

	SDRAM_DEVICE -> SDRTR = tmpreg;
}

/**
 * @fn void sdram_init(void)
 * @brief
 *
 * @pre
 * @post
 */
void sdram_init(void){
	sdram_command_t Command;

	Command.mode            = FMC_SDRAM_CMD_CLK_ENABLE;
	if(_conf -> bank == SDRAM_BANK1)
		Command.targer      = FMC_SDRAM_CMD_TARGET_BANK1;
	else
		Command.targer      = FMC_SDRAM_CMD_TARGET_BANK2;
	Command.autorefresh_num = 1;
	Command.registermode    = 0;
	fmc_sdram_sendcommand(Command);
	delay_ms(1);
	Command.mode            = FMC_SDRAM_CMD_PALL;
	fmc_sdram_sendcommand(Command);
	Command.mode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command.autorefresh_num      = 2;
	fmc_sdram_sendcommand(Command);
	Command.mode            = FMC_SDRAM_CMD_LOAD_MODE;
	Command.registermode =  (uint32_t)0 | 0<<3 | 2<<4 | 0<<7 | 1<<9;
	fmc_sdram_sendcommand(Command);
	/* COUNT = [(SDRAM self refresh time / number of row) x  SDRAM CLK] – 20
		  = [(64ms/4096) * 100MHz] - 20 = 1562.5 - 20 ~ 1542 */
	fmc_sdram_setrefreshrate(_conf -> refreshrate);
}


/**
 * @fn void fmc_sdram_hardware_init(void)
 * @brief
 *
 * @pre
 * @post
 */
void fmc_sdram_hardware_init(void){
	gpio_port_clock_enable(GPIOC);
	gpio_port_clock_enable(GPIOD);
	gpio_port_clock_enable(GPIOE);
	gpio_port_clock_enable(GPIOF);
	gpio_port_clock_enable(GPIOG);


/**
 * Initialize Address pin.
 */
	gpio_set_alternatefunction(FMC_A0_P, FMC_A0, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A0_P, FMC_A0, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A1_P, FMC_A1, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A1_P, FMC_A1, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A2_P, FMC_A2, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A2_P, FMC_A2, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A3_P, FMC_A3, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A3_P, FMC_A3, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A4_P, FMC_A4, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A4_P, FMC_A4, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A5_P, FMC_A5, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A5_P, FMC_A5, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A6_P, FMC_A6, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A6_P, FMC_A6, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A7_P, FMC_A7, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A7_P, FMC_A7, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A8_P, FMC_A8, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A8_P, FMC_A8, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A9_P, FMC_A9, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A9_P, FMC_A9, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A10_P, FMC_A10, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A10_P, FMC_A10, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_A11_P, FMC_A11, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_A11_P, FMC_A11, GPIO_OUTPUT_PUSHPULL);

/**
 * Initialize dataI/O pin.
 */
	gpio_set_alternatefunction(FMC_D0_P, FMC_D0, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D0_P, FMC_D0, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D1_P, FMC_D1, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D1_P, FMC_D1, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D2_P, FMC_D2, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D2_P, FMC_D2, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D3_P, FMC_D3, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D3_P, FMC_D3, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D4_P, FMC_D4, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D4_P, FMC_D4, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D5_P, FMC_D5, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D5_P, FMC_D5, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D6_P, FMC_D6, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D6_P, FMC_D6, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D7_P, FMC_D7, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D7_P, FMC_D7, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D8_P, FMC_D8, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D8_P, FMC_D8, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D9_P, FMC_D9, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D9_P, FMC_D9, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D10_P, FMC_D10, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D10_P, FMC_D10, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D11_P, FMC_D11, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D11_P, FMC_D11, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D12_P, FMC_D12, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D12_P, FMC_D12, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D13_P, FMC_D13, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D13_P, FMC_D13, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D14_P, FMC_D14, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D14_P, FMC_D14, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_D15_P, FMC_D15, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_D15_P, FMC_D15, GPIO_OUTPUT_PUSHPULL);

/**
 * Initialize control pin.
 */
	gpio_set_alternatefunction(FMC_NBL0_P, FMC_NBL0, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_NBL0_P, FMC_NBL0, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_NBL1_P, FMC_NBL1, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_NBL1_P, FMC_NBL1, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_BA0_P, FMC_BA0, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_BA0_P, FMC_BA0, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_BA1_P, FMC_BA1, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_BA1_P, FMC_BA1, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_CAS_P, FMC_CAS, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_CAS_P, FMC_CAS, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_RAS_P, FMC_RAS, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_RAS_P, FMC_RAS, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_CLK_P, FMC_CLK, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_CLK_P, FMC_CLK, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_NWE_P, FMC_NWE, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_NWE_P, FMC_NWE, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_NE_P, FMC_NE, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_NE_P, FMC_NE, GPIO_OUTPUT_PUSHPULL);

	gpio_set_alternatefunction(FMC_CKE_P, FMC_CKE, AF12_FSMC_SDIO_USB);
	gpio_set_alternatefunction_type(FMC_CKE_P, FMC_CKE, GPIO_OUTPUT_PUSHPULL);

}




#endif /* defined(STM32F429xx) */
#endif /* ENABLE_SDRAM */


