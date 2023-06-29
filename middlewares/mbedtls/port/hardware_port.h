/*
 * hardware_port.h
 *
 *  Created on: Jun 21, 2023
 *      Author: anh
 */

#ifndef MIDDLEWARES_MBEDTLS_PORT_HARDWARE_PORT_H_
#define MIDDLEWARES_MBEDTLS_PORT_HARDWARE_PORT_H_

#include "mbedtls/mbedtls_config.h"
#include "stdlib.h"

void *port_mbedtls_mem_calloc(size_t n, size_t size);
void port_mbedtls_mem_free(void *ptr);


#endif /* MIDDLEWARES_MBEDTLS_PORT_HARDWARE_PORT_H_ */
