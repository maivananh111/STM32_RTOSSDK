/*
 * http_auth.cpp
 *
 *  Created on: Jun 19, 2023
 *      Author: anh
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "sys/socket.h"
#include "system/log.h"
#include "system/error_check.h"

#include "protocols/tls/tls_crypto.h"
#include "protocols/http/http_com/http_utils.h"
#include "protocols/http/http_com/http_auth.h"

#include "mbedtls/md5.h"

#define MD5_MAX_LEN (33)
#define HTTP_AUTH_BUF_LEN (1024)

static const char *TAG = "HTTP_AUTH";

/**
 * @brief      This function hash a formatted string with MD5 and format the result as ascii characters
 *
 * @param      md         The buffer will hold the ascii result
 * @param[in]  fmt        The format
 *
 * @return     Length of the result
 */
static int md5_printf(char *md, const char *fmt, ...){
    unsigned char *buf;
    unsigned char digest[MD5_MAX_LEN];
    size_t len;
    int i;
    mbedtls_md5_context md5_ctx;
    va_list ap;
    va_start(ap, fmt);
    len = vasprintf((char **)&buf, fmt, ap);
    if (buf == NULL) {
        va_end(ap);
        return -1;
    }

    mbedtls_md5_init(&md5_ctx);
    mbedtls_md5_starts(&md5_ctx);

    len = strlen((char *)buf);
    mbedtls_md5_update(&md5_ctx, reinterpret_cast<const unsigned char*>(buf), len);

	mbedtls_md5_finish(&md5_ctx, digest);
	mbedtls_md5_free(&md5_ctx);

    for(i = 0; i < 16; ++i) {
        sprintf(&md[i * 2], "%02x", (unsigned int)digest[i]);
    }
    va_end(ap);

    free(buf);
    return MD5_MAX_LEN;
}

char *http_auth_digest(const char *username, const char *password, http_auth_data_t *auth_data){
    char *ha1, *ha2 = NULL;
    char *digest = NULL;
    char *auth_str = NULL;
    char *temp_auth_str = NULL;
    error_t ret = E_OKE;
    int rc;

    if (username == NULL ||
        password == NULL ||
        auth_data->nonce == NULL ||
        auth_data->uri == NULL ||
        auth_data->realm == NULL) {
        return NULL;
    }

    ha1 = (char *)calloc(1, MD5_MAX_LEN);
    CHECK_GOTO((ha1 == NULL), _digest_exit, LOG_ERROR, TAG, "Memory exhausted");

    ha2 = (char *)calloc(1, MD5_MAX_LEN);
    CHECK_GOTO((ha2 == NULL), _digest_exit, LOG_ERROR, TAG, "Memory exhausted");

    digest = (char *)calloc(1, MD5_MAX_LEN);
    CHECK_GOTO((digest == NULL), _digest_exit, LOG_ERROR, TAG, "Memory exhausted");


    if(md5_printf(ha1, "%s:%s:%s", username, auth_data->realm, password) <= 0)
    	goto _digest_exit;

    LOG_DEBUG(TAG, "%s %s %s %s", "Digest", username, auth_data->realm, password);
    if(strcasecmp(auth_data->algorithm, "md5-sess") == 0) {
        if(md5_printf(ha1, "%s:%s:%016llx", ha1, auth_data->nonce, auth_data->cnonce) <= 0)
            goto _digest_exit;
    }
    if(md5_printf(ha2, "%s:%s", auth_data->method, auth_data->uri) <= 0)
    		goto _digest_exit;

    if(auth_data->qop && strcasecmp(auth_data->qop, "auth-int") == 0) {
        if (md5_printf(ha2, "%s:%s", ha2, "entity") <= 0)
            goto _digest_exit;
    }

    if(auth_data->qop) {
        if (md5_printf(digest, "%s:%s:%08x:%016llx:%s:%s", ha1, auth_data->nonce, auth_data->nc, auth_data->cnonce, auth_data->qop, ha2) <= 0)
            goto _digest_exit;
    }
    else{
        if (md5_printf(digest, "%s:%s:%s", ha1, auth_data->nonce, ha2) <= 0)
            goto _digest_exit;
    }
    rc = asprintf(&auth_str, "Digest username=\"%s\", realm=\"%s\", nonce=\"%s\", uri=\"%s\", algorithm=\"MD5\", "
             "response=\"%s\", qop=%s, nc=%08x, cnonce=%016" PRIx64,
             username, auth_data->realm, auth_data->nonce, auth_data->uri, digest, auth_data->qop, auth_data->nc, auth_data->cnonce);
    if(rc < 0){
        LOG_ERROR(TAG, "asprintf() returned: %d", rc);
        ret = E_FAIL;
        goto _digest_exit;
    }
    if(auth_data->opaque){
        rc = asprintf(&temp_auth_str, "%s, opaque=\"%s\"", auth_str, auth_data->opaque);
        free(auth_str);
        if(rc < 0){
        	LOG_ERROR(TAG, "asprintf() returned: %d", rc);
            ret = E_FAIL;
            goto _digest_exit;
        }
        auth_str = temp_auth_str;
    }

    _digest_exit:
    free(ha1);
    free(ha2);
    free(digest);

    return (ret == E_OKE)? auth_str : NULL;
}

char *http_auth_basic(const char *username, const char *password){
    size_t out;
    char *user_info = NULL;
    char *digest = NULL;
    error_t ret = E_OKE;
    size_t n = 0;

    if (asprintf(&user_info, "%s:%s", username, password) < 0) {
        return NULL;
    }

    CHECK_GOTO((user_info == NULL), _basic_exit, LOG_ERROR, TAG, "Memory exhausted");
    crypto_base64_encode(NULL, 0, &n, (const unsigned char *)user_info, strlen(user_info));

    digest = (char *)calloc(1, 6 + n + 1);
    CHECK_GOTO((digest == NULL), _basic_exit, LOG_ERROR, TAG, "Memory exhausted");
    strcpy(digest, "Basic ");
    crypto_base64_encode((unsigned char *)digest + 6, n, &out, (const unsigned char *)user_info, strlen(user_info));

    _basic_exit:
    free(user_info);

    return (ret == E_OKE)? digest : NULL;
}
