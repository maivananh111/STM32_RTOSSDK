/*
 * crc.h
 *
 *  Created on: Apr 29, 2023
 *      Author: anh
 */

#ifndef COMPONENTS_CRC_H_
#define COMPONENTS_CRC_H_

#include "component_config.h"
#if ENABLE_COMPONENT_CRC

#include "stdio.h"
#include "stdlib.h"

#ifdef __cplusplus
extern "C"{
#endif

uint16_t cal_crc16(uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_CRC */

#endif /* COMPONENTS_CRC_H_ */
