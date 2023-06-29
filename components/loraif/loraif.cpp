/*
 * loraif.cpp
 *
 *  Created on: Apr 29, 2023
 *      Author: anh
 */

#include "component_config.h"
#if ENABLE_COMPONENT_LORAIF

#include "loraif/loraif.h"
#include "parse_packet/parse_packet.h"
#include "crc/crc.h"

#include "stdlib.h"
#include "string.h"

#include "system/log.h"
#include "system/system.h"
#include "periph/systick.h"
#include "periph/rng.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

using namespace std;


#if ENABLE_COMPONENT_LORAIF_DEBUG
static const char *TAG = "LoRaIF";
#define LOG_LEVEL LOG_DEBUG
#endif /* ENABLE_COMPONENT_LORAIF_DEBUG */
static const char *command_string[] = {
	"LORA_ERR",

	"LORA_REQ_ADDRESS",
	"LORA_UPDATE_ADDRESS",

	"LORA_REQ_DATA",
	"LORA_RES_DATA",
	"LORA_UPDATE_DATA",

	"LORA_ADD_DEVICE",
	"LORA_REMOVE_DEVICE",
	"LORA_DEVICE_NOT_RESPONSE",

	"LORA_CMD_NUM",
};

sx127x *loraif;
static void (*fpeventhandler)(lora_event_t event, uint32_t device_address, char *data);

typedef struct {
	uint32_t address;
	uint32_t tick_start = 0;
} loraif_request_prop_t;

static uint32_t addr_by_gw = 0x00;
static uint32_t resp_timeout;
static uint8_t max_not_resp;
static uint8_t _send_syncword, _recv_syncword;

static QueueHandle_t q_wait_response, q_response;
static SemaphoreHandle_t s_lora_transfer;
list<loraif_dev_t *> loraif_device_list;

static bool loraif_transmit(char *str);
static void loraif_debug(char *str, int line, const char *func);
static void show_device_list(void);
static bool device_is_available(uint32_t num);


static void loraif_debug(char *str, int line, const char *func){
#if ENABLE_COMPONENT_LORAIF_DEBUG
	LOG_LEVEL(TAG, "%s, Line: %d Function: %s", str, line, func);
#endif /* ENABLE_COMPONENT_LORAIF_DEBUG */
}

static void show_device_list(void){
	if(loraif_device_list.empty()){
		LOG_WARN(TAG, "Device list empty, notthing to show.");
		return;
	}

	uint8_t i = 1;
    for (auto device = loraif_device_list.begin(); device != loraif_device_list.end(); ++device) {
    	LOG_WARN(TAG, "Device %02d address: 0x%08x name: \"%s\".", i++, (unsigned int)(*device)->address, (*device)->name);
    }
}

static bool device_is_available(uint32_t num){
    for (auto device = loraif_device_list.begin(); device != loraif_device_list.end(); ++device) {
        if ((*device)->address == num) {
            return true;
        }
    }
    return false;
}

static bool loraif_transmit(char *str){
	if(xSemaphoreTake(s_lora_transfer, 10)){
		loraif->setSyncWord(_send_syncword);
		loraif->beginPacket();
		loraif->transmit((uint8_t *)str, (size_t)strlen(str));
		loraif->endPacket();
//		LOG_LEVEL(TAG, str);
		loraif->setSyncWord(_recv_syncword);
		loraif->receive_it(0);
		xSemaphoreGive(s_lora_transfer);
		return true;
	}

	return false;
}

static void set_response_ok(uint32_t addr, lora_event_t cmd){
	uint16_t crc = 0;
	char *temp = NULL;
	char *response_to_device = NULL;
	char *cmd_str = cmd_to_str(cmd, command_string);

	asprintf(&temp, "%s: {\"addr\":0x%08x,\"state\":OK,", cmd_str, (unsigned int)addr);
	crc = cal_crc16((uint8_t *)temp, strlen(temp));
	if(temp != NULL) free(temp);

	asprintf(&response_to_device, "%s: {\"addr\":0x%08x,\"state\":OK,\"crc\":0x%04x}", cmd_str, (unsigned int)addr, crc);

	if(xQueueSend(q_response, &response_to_device, 10) == pdFALSE){
		loraif_debug((char *)"Can't send to q_response", __LINE__, __FUNCTION__);
	}
}


void loraif_init(sx127x *lora, uint8_t send_syncword, uint8_t recv_syncword, uint32_t timeout, uint8_t max_not_response){
	loraif = lora;
	resp_timeout = timeout;
	max_not_resp = max_not_response;
	_send_syncword = send_syncword;
	_recv_syncword = recv_syncword;

	q_wait_response = xQueueCreate(LORAIF_QUEUE_SIZE, sizeof(loraif_request_prop_t *));
	q_response = xQueueCreate(LORAIF_QUEUE_SIZE, sizeof(char *));

	s_lora_transfer = xSemaphoreCreateBinary();
	xSemaphoreGive(s_lora_transfer);

	loraif->setSyncWord(_recv_syncword);
	loraif->receive_it(0);
}

void loraif_register_event_handler(void (*peventhandler)(lora_event_t event, uint32_t device_address, char *data)){
	fpeventhandler = peventhandler;
}

bool loraif_check_receive_data_crc(char *data){
	char *src_cpy = data;
	char *crc_start;
	uint16_t crc, icrc, len;
	pkt_t pkt;
	pkt_err_t err;
	pkt_json_t json;

	crc_start = strstr(src_cpy, "\"crc\"");
	if(crc_start == NULL) return false;

	err = parse_packet(src_cpy, &pkt);
	if(err != PKT_ERR_OK) return false;
	err = json_get_object(pkt.data_str, &json, (char *)"crc");
	if(err != PKT_ERR_OK) return false;
	icrc = strtol(json.value, NULL, 16);
	json_release_object(&json);
	release_packet(&pkt);

	len = (uint32_t)crc_start - (uint32_t)src_cpy;
	crc = cal_crc16((uint8_t *)src_cpy, len);
	if(crc != icrc) return false;

	return true;
}

bool loraif_isvalid_address(uint32_t address){
	return (address != LORAIF_INVALID_ADDRESS);
}


void loraif_send_request(uint32_t dev_address, lora_event_t cmd, char *data, int require_resp){
	uint16_t crc = 0;
	char *req_data = NULL, *temp = NULL;

	char *cmd_str = cmd_to_str(cmd, command_string);
	asprintf(&temp, "%s: {\"addr\":0x%08x,\"data\":%s,\"require_response\":%d,", cmd_str, (unsigned int)dev_address, data, require_resp);
	crc = cal_crc16((uint8_t *)temp, strlen(temp));
	if(temp != NULL) free(temp);

	asprintf(&req_data, "%s: {\"addr\":0x%08x,\"data\":%s,\"require_response\":%d,\"crc\":0x%04x}", cmd_str, (unsigned int)dev_address, data, require_resp, crc);

	loraif_dev_t *dev = loraif_select_device(dev_address);
	if(dev != NULL && dev->name != NULL){
		if(require_resp != 0){
			/** Create new require response profile */
			loraif_request_prop_t *wait_response = (loraif_request_prop_t *)malloc(sizeof(loraif_request_prop_t));
			if(wait_response == NULL){
				loraif_debug((char *)"Memory allocation fail", __LINE__, __FUNCTION__);
			}
			wait_response->address = dev_address;
			wait_response->tick_start = get_tick();
			/** Send wait_response to_queue */
			loraif_request_prop_t *in_queue = NULL;
			uint8_t queue_len = uxQueueMessagesWaiting(q_wait_response);
			for(uint8_t i=0; i<queue_len; i++){
				if(xQueueReceive(q_wait_response, &in_queue, 10) == pdTRUE && in_queue != NULL){
					if(in_queue->address == wait_response->address){
						if(wait_response != NULL) free(wait_response);
						wait_response = NULL;
					}
					if(xQueueSend(q_wait_response, &in_queue, 10) != pdTRUE){
						loraif_debug((char *)"Can't send to q_wait_response", __LINE__, __FUNCTION__);
					}
				}
			}
			if(wait_response != NULL){
				if(xQueueSend(q_wait_response, &wait_response, 10) != pdTRUE){
					loraif_debug((char *)"Can't send to q_wait_response", __LINE__, __FUNCTION__);
				}
			}
		}
		if(xQueueSend(q_response, &req_data, 10) == pdFALSE){
			loraif_debug((char *)"Can't send data to device", __LINE__, __FUNCTION__);
		}
	}
	else{
		if(req_data != NULL) free(req_data);
	}

//	if(loraif_transmit(req_data) != true){
//		LOG_ERROR(TAG, "LoRa busy.");
//	}
//	if(req_data != NULL) free(req_data);
}


void loraif_receive_process(void *param){
	QueueHandle_t *queue = (QueueHandle_t *)param;
	char *rx_full;

	if(xQueueReceive(*queue, &rx_full, 10)){
		pkt_err_t err;
		pkt_t pkt;
		pkt_json_t json;
		lora_event_t cmd;
		uint32_t addr = LORAIF_INVALID_ADDRESS;
		char *evt_data = NULL;
		char *response_to_device = NULL;

		err = parse_packet(rx_full, &pkt);
/** Parse packet success */
		if(err == PKT_ERR_OK){
			cmd = (lora_event_t)str_to_cmd(pkt.cmd_str, command_string, (int)LORA_CMD_NUM);

		/** Response without device address */
			if(cmd == LORA_REQ_ADDRESS){
				err = json_get_object(pkt.data_str, &json, (char *)"key");

				if(err == PKT_ERR_OK){
					/** Generate new address */
					uint32_t rand_num = strtol(json.value, NULL, 16);

					rng_set_seed(rand_num);
					while(1){
						addr_by_gw = rng_generate_random_number();
					    if(device_is_available(addr_by_gw)) continue;
					    else break;
					}
					addr_by_gw &= 0x7FFFFFFFU;
					/** Response new address to new device */
					uint16_t crc = 0;
					char *temp = NULL;

					asprintf(&temp, "LORA_RES_ADDRESS: {\"addr\":0x%08x,\"key\":0x%08x,", (unsigned int)addr_by_gw, (unsigned int)rand_num);
					crc = cal_crc16((uint8_t *)temp, strlen(temp));
					if(temp != NULL) free(temp);

					asprintf(&response_to_device, "LORA_RES_ADDRESS: {\"addr\":0x%08x,\"key\":0x%08x,\"crc\":0x%04x}", (unsigned int)addr_by_gw, (unsigned int)rand_num, crc);
					if(xQueueSend(q_response, &response_to_device, 10) == pdFALSE){
						loraif_debug((char *)"Can't send address to new device", __LINE__, __FUNCTION__);
					}

					goto event_handle;
				}
				else{
					loraif_debug((char *)"Can't allocate new device address", __LINE__, __FUNCTION__);
				}

				json_release_object(&json);
			}

		/** Response with device address */
			else{
				err = json_get_object(pkt.data_str, &json, (char *)"addr");
				addr = strtol(json.value, NULL, 16);

				if(err == PKT_ERR_OK){
					if(cmd == LORA_ERR){
						loraif_debug((char *)"Device error", __LINE__, __FUNCTION__);
						cmd = LORA_ERR;
					}

					else if(cmd == LORA_UPDATE_ADDRESS){
						if(addr == addr_by_gw){
							set_response_ok(addr, cmd);
							cmd = LORA_ADD_DEVICE;
							goto event_handle;
						}
					}

					/** LORA_UPDATE_SETTINGS, LORA_RES_DATA, LORA_UPDATE_DATA */
					else{
						if(cmd == LORA_RES_DATA) cmd = LORA_REQ_DATA;
						bool valid_addr = false;
						loraif_request_prop_t *require_resp_prop = NULL;

						for(uint8_t i=0; i<uxQueueMessagesWaiting(q_wait_response); i++){
							if(xQueueReceive(q_wait_response, &require_resp_prop, 10) == pdTRUE && require_resp_prop != NULL){
								if(require_resp_prop->address == addr){
									free(require_resp_prop);
									require_resp_prop = NULL;
									valid_addr = true;
									break;
								}
								else{
									if(xQueueSend(q_wait_response, &require_resp_prop, 10) != pdTRUE){
										loraif_debug((char *)"Can't send to q_wait_response", __LINE__, __FUNCTION__);
									}
								}
							}
						}

						if(valid_addr){
							loraif_dev_t *err_dev = loraif_select_device(addr);
							if(err_dev == NULL){
								cmd = LORA_ERR;
								goto event_handle;
							}
							err_dev->err_count = 0;
							set_response_ok(addr, cmd);
							goto event_handle;
						}

					}
				}
				else{
					loraif_debug((char *)"Unknown device", __LINE__, __FUNCTION__);
					cmd = LORA_ERR;
				}

				json_release_object(&json);
			}

			event_handle:
			asprintf(&evt_data, "%s", pkt.data_str);
			if(fpeventhandler != NULL) fpeventhandler(cmd, addr, evt_data);
			if(evt_data != NULL) free(evt_data);
		}

/** Parse packet error */
		else{
			loraif_debug((char *)"Can't parse received packet", __LINE__, __FUNCTION__);
		}

		json_release_object(&json);
		release_packet(&pkt);
		if(rx_full != NULL) free(rx_full);
	}
}


void loraif_response_to_device(void*){
	char *response;
	while(1){
		if(xQueueReceive(q_response, &response, 10) && response != NULL){
			if(loraif_transmit(response) == true)
				free(response);
			else{
				if(xQueueSend(q_response, &response, 10) != pdTRUE){
					loraif_debug((char *)"Can't send to q_response", __LINE__, __FUNCTION__);
				}
			}
		}
	}
}

void loraif_check_device_timeout(void){
	loraif_request_prop_t *wait_response = NULL;

	uint8_t queue_len = uxQueueMessagesWaiting(q_wait_response);

	for(uint8_t i=0; i<queue_len; i++){
		if(xQueueReceive(q_wait_response, &wait_response, 10) == pdTRUE && wait_response != NULL){
			uint32_t dt = 0, tick_now = get_tick();
			if(tick_now >= wait_response->tick_start) dt = tick_now - wait_response->tick_start;
			else                                      dt = (4294967295 - wait_response->tick_start) + tick_now;

			if(dt >= resp_timeout){
				loraif_dev_t *err_dev = loraif_select_device(wait_response->address);
				if(err_dev != NULL){
					char *evt_data = NULL;
					asprintf(&evt_data, "{\"addr\":0x%08x,\"name\":\"%s\"}", (unsigned int)wait_response->address, err_dev->name);
					err_dev->err_count++;

					LOG_RET(TAG, "Device 0x%08x time = %lu, %d time not response.", (unsigned int)wait_response->address, dt, err_dev->err_count);

					if(fpeventhandler != NULL) fpeventhandler(LORA_DEVICE_NOT_RESPONSE, wait_response->address, evt_data);
					if(err_dev->err_count >= max_not_resp){
						if(fpeventhandler != NULL) fpeventhandler(LORA_REMOVE_DEVICE, wait_response->address, evt_data);
					}

					if(evt_data != NULL) free(evt_data);
					if(wait_response != NULL) free(wait_response);
					wait_response = NULL;
				}
			}
			else if(dt < resp_timeout){
				if(xQueueSend(q_wait_response, &wait_response, 10) != pdTRUE){
					loraif_debug((char *)"Can't send to q_wait_response", __LINE__, __FUNCTION__);
				}
			}
		}
	}
}


void loraif_add_device(uint32_t device_address, char *jdata, void *dev_data){

	pkt_err_t err;
	pkt_json_t json;
    loraif_dev_t *newdev = (loraif_dev_t *)malloc(sizeof(loraif_dev_t));

    newdev->address = device_address;
    newdev->err_count = 0;
    newdev->data = dev_data;

	err = json_get_object(jdata, &json, (char *)"name");
	if(err == PKT_ERR_OK)
		asprintf(&newdev->name, "%s", json.value);
	json_release_object(&json);

	loraif_device_list.push_back(newdev);

	show_device_list();
}

void loraif_remove_device(uint32_t device_address){

    if (loraif_device_list.empty()) {
    	loraif_debug((char *)"Device list empty", __LINE__, __FUNCTION__);
        return;
    }

    auto device = loraif_device_list.begin();
    while (device != loraif_device_list.end()) {
        if ((*device)->address == device_address) {
            break;
        }
        ++device;
    }

    if(device == loraif_device_list.end()) {
    	loraif_debug((char *)"This device not available in device list", __LINE__, __FUNCTION__);
        return;
    }

    if((*device)->name != NULL) free((*device)->name);
    loraif_device_list.erase(device);
    if((*device) != NULL) free((*device));

	show_device_list();
}

loraif_dev_t *loraif_select_device(uint32_t device_address){

    if (loraif_device_list.empty()) {
    	loraif_debug((char *)"Device list empty", __LINE__, __FUNCTION__);
        return NULL;
    }

    auto device = loraif_device_list.begin();
    while (device != loraif_device_list.end()) {
        if ((*device)->address == device_address) {
            break;
        }
        ++device;
    }

    if (device == loraif_device_list.end()) {
    	loraif_debug((char *)"This device not available in device list", __LINE__, __FUNCTION__);
        return NULL;
    }

    return (*device);

}




#endif /* ENABLE_COMPONENT_WIFIIF */
