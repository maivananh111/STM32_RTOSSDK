/*
 * main.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef STARTUP_H_
#define STARTUP_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "peripheral_config.h"
#include "system/system.h"
#include "periph/systick.h"
#include "periph/rcc.h"
#include "periph/gpio.h"

#if CONFIG_USE_SDRAM && ENABLE_FMC
#include "periph/fmc_sdram.h"
#endif /* CONFIG_USE_SDRAM && ENABLE_FMC */

#if CONFIG_RTOS_USE_IWDG && ENABLE_IWDG
#include "periph/iwdg.h"
#endif /* CONFIG_RTOS_USE_IWDG && ENABLE_IWDG */

#if ENABLE_RNG && defined(RNG)
#include "periph/rng.h"
#endif /* ENABLE_RNG && defined(RNG) */

#if CONFIG_USE_LOG_MONITOR
#include "system/log.h"
#endif /* CONFIG_USE_LOG_MONITOR */

#if CONFIG_USE_LOG_MONITOR && ENABLE_USB && CONFIG_LOG_OVER_USB
#include "usb_device/usb_device.h"
#include "usb_device/usb_cdc/usbd_cdc_if.h"
static void usb_log(char *log);
#endif
#if CONFIG_USE_LOG_MONITOR && ENABLE_USART && CONFIG_LOG_OVER_UART
static void uart_log_init(void);
static void uart_log(char *log);
#endif

void app_main_task(void *);
int main_application(void);

#ifdef __cplusplus
}
#endif

#endif /* STARTUP_H_ */
