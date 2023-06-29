/*
 * button.cpp
 *
 *  Created on: May 12, 2023
 *      Author: anh
 */
#include "component_config.h"
#if ENABLE_COMPONENT_BUTTON

#include "button/button.h"
#include "periph/gpio.h"
#include "periph/exti.h"


#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"



static EventGroupHandle_t btn_event_bits = NULL;
void button_init(button_t *btn){
	gpio_port_clock_enable(btn->port);

	if(btn->trigger_type == BUTTON_TRIGGER_POLLING){
		if(btn->active_level == BUTTON_ACTIVE_LOW)
			gpio_set_mode(btn->port, btn->pin, GPIO_INPUT_PULLUP);
		if(btn->active_level == BUTTON_ACTIVE_HIGH)
			gpio_set_mode(btn->port, btn->pin, GPIO_INPUT_PULLDOWN);
	}
	else{
		gpio_set_mode(btn->port, btn->pin, GPIO_INPUT_PULL);
		if(btn->active_level == BUTTON_ACTIVE_LOW){
			exti_init(btn->port, btn->pin, EXTI_FALLING_EDGE, btn->interruptpriority);
			gpio_set_pullup(btn->port, btn->pin);
		}
		if(btn->active_level == BUTTON_ACTIVE_HIGH){
			exti_init(btn->port, btn->pin, EXTI_RISING_EDGE, btn->interruptpriority);
			gpio_set_pulldown(btn->port, btn->pin);
		}
	}

	if(btn_event_bits == NULL) btn_event_bits = xEventGroupCreate();
}

void button_task_poll_handler(button_t *btn){
	button_event_t event = BUTTON_RELEASE;
	if(gpio_get_level(btn->port, btn->pin) == btn->active_level){
		for(uint16_t i=0; i<btn->longpress_time; i++){
			btn->active_count++;
			vTaskDelay(1);
			if(gpio_get_level(btn->port, btn->pin) != btn->active_level) break;
		}
		if(btn->active_count == btn->longpress_time){
			event = BUTTON_LONGPRESS;
			goto eventcb;
		}
		else if(gpio_get_level(btn->port, btn->pin) != btn->active_level){
			btn->inactive_count = 0;
			for(uint16_t j=0; j<btn->doubleclick_release_time; j++){
				btn->inactive_count++;
				vTaskDelay(1);
				if(gpio_get_level(btn->port, btn->pin) == btn->active_level) break;
			}
			if(btn->inactive_count < btn->doubleclick_release_time){
				event = BUTTON_DOUBLECLICK;
				goto eventcb;
			}
			else if(btn->inactive_count == btn->doubleclick_release_time){
				event = BUTTON_SINGLECLICK;
				goto eventcb;
			}
		}

		eventcb:
		if((btn->event_option & event) && (btn->event_handler != NULL)) btn->event_handler(event);

		btn->active_count = 0;
		btn->inactive_count = 0;

		return;
	}
}

void button_task_handler(button_t *btn){
	EventBits_t bit = xEventGroupWaitBits(btn_event_bits, (1<<btn->bits), pdTRUE, pdFALSE, 5);
	if(bit != 0U && bit == (EventBits_t)(1<<btn->bits)){
		button_task_poll_handler(btn);
		xEventGroupClearBits(btn_event_bits, (1<<btn->bits));
		btn->isactive = false;
	}
}

void button_interrupt_trigger(button_t *btn){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(btn->isactive == false){
		xEventGroupSetBitsFromISR(btn_event_bits, (1<<btn->bits), &xHigherPriorityTaskWoken);
		btn->isactive = true;
	}
}

#endif /* ENABLE_COMPONENT_BUTTON */
