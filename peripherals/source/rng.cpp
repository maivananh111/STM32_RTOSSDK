/*
 * rng.cpp
 *
 *  Created on: Apr 2, 2023
 *      Author: anh
 */

#include "peripheral_config.h"

#if ENABLE_RNG && defined(RNG)
#include "periph/rng.h"

#include "periph/systick.h"
#include "system/system.h"


uint32_t _seed = 0;
void rng_init(void){
	RCC -> AHB2ENR |= RCC_AHB2ENR_RNGEN;

	RNG -> CR |=  RNG_CR_RNGEN;

	rng_set_seed(sys_get_free_heap_size());
}

uint32_t rng_random(void){
	__IO uint32_t tick = get_tick();
	__IO uint32_t random_number = 0U;

	while(!(RNG -> SR & RNG_SR_DRDY)){
		if(get_tick() - tick > RNG_TIMEOUT){
			break;
		}
	}
	random_number = RNG -> DR;

	return random_number;
}

uint32_t rng_random_invert(void){
	return ~rng_random();
}

void rng_set_seed(uint32_t seed){
	_seed = seed;
}

uint32_t rng_generate_random_number(void){
	__IO uint32_t rand = _seed;
	for(int i=0; i<2; i++){
		rand ^= rng_random();
		rand ^= rng_random_invert();
	}
	return rand;
}






#endif /* ENABLE_RNG && defined(RNG) */
