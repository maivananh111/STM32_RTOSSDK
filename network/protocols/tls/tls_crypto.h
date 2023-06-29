/*
 * tls_crypto.h
 *
 *  Created on: Jun 19, 2023
 *      Author: anh
 */


#ifndef TLS_TLS_CRYPTO_H
#define TLS_TLS_CRYPTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>


int crypto_sha1(const unsigned char *input, size_t ilen, unsigned char output[20]);

int crypto_base64_encode(unsigned char *dst, size_t dlen, size_t *olen, const unsigned char *src, size_t slen);

#ifdef __cplusplus
}
#endif
#endif /* TLS_TLS_CRYPTO_H */
