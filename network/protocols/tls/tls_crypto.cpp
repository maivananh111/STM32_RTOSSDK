/*
 * tls_crypto.cpp
 *
 *  Created on: Jun 19, 2023
 *      Author: anh
 */

#include "protocols/tls/tls_crypto.h"

#include "mbedtls/sha1.h"
#include "mbedtls/base64.h"

int crypto_sha1(const unsigned char *input, size_t ilen, unsigned char output[20]){
    mbedtls_sha1(input, ilen, output);

    return 1;
}

int crypto_base64_encode(unsigned char *dst, size_t dlen, size_t *olen, const unsigned char *src, size_t slen ){
	return mbedtls_base64_encode(dst, dlen, olen, src, slen);
}
