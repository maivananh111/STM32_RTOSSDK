/*
 * hardware_port.c
 *
 *  Created on: Jun 21, 2023
 *      Author: anh
 */

#include "hardware_port.h"

#ifdef MBEDTLS_ENTROPY_HARDWARE_ALT

#include "string.h"
#include "entropy_poll.h"
#include "periph/rng.h"

#if !defined(MIN)
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

static void fill_random(void *buf, size_t len){
    uint8_t *buf_bytes = (uint8_t *)buf;

    while (len > 0) {
        uint32_t word = rng_generate_random_number();
        uint32_t to_copy = MIN(sizeof(word), len);
        memcpy(buf_bytes, &word, to_copy);
        buf_bytes += to_copy;
        len -= to_copy;
    }
}

int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen ){
	fill_random(output, len);
    *olen = len;
    return 0;
}

#endif /*MBEDTLS_ENTROPY_HARDWARE_ALT*/


#ifdef MBEDTLS_PLATFORM_MEMORY
#include "FreeRTOS.h"

void *port_mbedtls_mem_calloc(size_t n, size_t size){
	void * x = pvPortMalloc(size);
	memset(x, 0, size);
	return x;
}

void port_mbedtls_mem_free(void *ptr){
	vPortFree(ptr);
}

#endif /* MBEDTLS_PLATFORM_MEMORY */


