/*
 * button.h
 *
 *  Created on: May 12, 2023
 *      Author: anh
 */

#ifndef COMPONENTS_BUTTON_H_
#define COMPONENTS_BUTTON_H_

#include "component_config.h"
#if ENABLE_COMPONENT_BUTTON

#include "sdkconfig.h"
#include st_header
#include "stdio.h"


typedef enum{
	BUTTON_RELEASE = 0,
	BUTTON_SINGLECLICK = 0x01,
	BUTTON_DOUBLECLICK = 0x02,
	BUTTON_LONGPRESS = 0x04,
	BUTTON_FULLOPTION = 0x07,
} button_event_t;

typedef enum{
	BUTTON_TRIGGER_POLLING,
	BUTTON_TRIGGER_INTERRUPT,
} button_trigger_t;

typedef enum{
	BUTTON_ACTIVE_LOW = 0,
	BUTTON_ACTIVE_HIGH = 1,
} button_activelevel_t;


typedef struct{
	GPIO_TypeDef *port = NULL;
	uint16_t pin = 0;
	button_trigger_t trigger_type = BUTTON_TRIGGER_POLLING;
	button_activelevel_t active_level = BUTTON_ACTIVE_LOW;
	uint32_t interruptpriority = 0;
	button_event_t event_option = BUTTON_SINGLECLICK;
	uint16_t longpress_time = 200U;
	uint16_t doubleclick_release_time = 150U;
	uint16_t bits = 0;
	void (*event_handler)(button_event_t);
	/**
	 * private.
	 */
	uint32_t active_count = 0;
	uint32_t inactive_count = 0;
	bool isactive = false;
} button_t;

void button_init(button_t *btn);

void button_task_handler(button_t *btn);
void button_task_poll_handler(button_t *btn);
void button_interrupt_trigger(button_t *btn);





#endif /* ENABLE_COMPONENT_BUTTON */

#endif /* COMPONENTS_BUTTON_H_ */
