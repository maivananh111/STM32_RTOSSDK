/*
 * rng.h
 *
 *  Created on: Apr 2, 2023
 *      Author: anh
 */

#ifndef PERIPH_RNG_H_
#define PERIPH_RNG_H_

#include "peripheral_config.h"

#if ENABLE_RNG && defined(RNG)

#ifdef __cplusplus
extern "C" {
#endif

void rng_init(void);

uint32_t rng_random(void);
uint32_t rng_random_invert(void);
void rng_set_seed(uint32_t seed);
uint32_t rng_generate_random_number(void);


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_RNG && defined(RNG) */

#endif /* PERIPH_RNG_H_ */
