/*
 * kalman_filter.h
 *
 *  Created on: May 16, 2023
 *      Author: anh
 */

#ifndef COMPONENTS_KALMAN_FILTER_H_
#define COMPONENTS_KALMAN_FILTER_H_

#include "component_config.h"
#if ENABLE_COMPONENT_KALMAN_FILER

#ifdef __cplusplus
extern "C"{
#endif

#include <math.h>
#include "stdint.h"

class kalmanfilter{
	public:
		kalmanfilter(float mea_e, float est_e, float q);
		float update_estimate(float mea, uint8_t loop);
		void set_measurement_error(float mea_e);
		void set_estimate_error(float est_e);
		void set_process_noise(float q);
		float get_gain(void);
		float get_estimate_error(void);

	private:
		float _err_measure;
		float _err_estimate;
		float _q;
		float _current_estimate = 0;
		float _last_estimate = 0;
		float _kalman_gain = 0;
};


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_KALMAN_FILER */
#endif /* COMPONENTS_KALMAN_FILTER_H_ */
