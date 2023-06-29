/*
 * loraif.h
 *
 *  Created on: Apr 29, 2023
 *      Author: anh
 */

#ifndef COMPONENTS_LORAIF_H_
#define COMPONENTS_LORAIF_H_

#include "component_config.h"
#if ENABLE_COMPONENT_LORAIF

#include "sdkconfig.h"
#include st_header
#include "stdio.h"
#include "list"
#include "algorithm"

#include "parse_packet/parse_packet.h"

#include "sx127x/sx127x.h"

using namespace std;

#define LORAIF_QUEUE_SIZE 30
#define LORAIF_INVALID_ADDRESS 0x00000000U
typedef enum {
	LORA_ERR,

	/**
	 * Gateway passive.
	 */
	LORA_REQ_ADDRESS, // No address.
	LORA_UPDATE_ADDRESS,

	/**
	 * Gateway proactive.
	 */
	LORA_REQ_DATA,
	LORA_RES_DATA,
	LORA_UPDATE_DATA,

	LORA_ADD_DEVICE,
	LORA_REMOVE_DEVICE,
	LORA_DEVICE_NOT_RESPONSE,

	LORA_CMD_NUM,
} lora_event_t;


typedef struct{
	uint32_t address;
	char *name;
	uint8_t err_count;
	void *data;
} loraif_dev_t;

extern list<loraif_dev_t *> loraif_device_list;

#ifdef __cplusplus
extern "C"{
#endif

void loraif_init(sx127x *lora, uint8_t send_syncword, uint8_t recv_syncword, uint32_t timeout, uint8_t max_not_response);
void loraif_register_event_handler(void (*peventhandler)(lora_event_t event, uint32_t device_address, char *data));

bool loraif_check_receive_data_crc(char *data);
bool loraif_isvalid_address(uint32_t address);

void loraif_send_request(uint32_t dev_address, lora_event_t cmd, char *data, int require_resp);
void loraif_receive_process(void *param);
void loraif_response_to_device(void*);
void loraif_check_device_timeout(void);

void loraif_add_device(uint32_t device_address, char *jdata, void *dev_data);
void loraif_remove_device(uint32_t device_address);
loraif_dev_t *loraif_select_device(uint32_t device_address);


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_WIFIIF */

#endif /* COMPONENTS_LORAIF_H_ */
