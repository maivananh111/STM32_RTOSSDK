/*
 * kalman_filter.cpp
 *
 *  Created on: May 16, 2023
 *      Author: anh
 */

#include "component_config.h"
#if ENABLE_COMPONENT_KALMAN_FILER

#include "kalman_filter/kalman_filter.h"



kalmanfilter::kalmanfilter(float mea_e, float est_e, float q){
	_err_measure=mea_e;
	_err_estimate=est_e;
	_q = q;
}

float kalmanfilter::update_estimate(float mea, uint8_t loop){
	for(uint8_t i=0; i<loop; i++){
		_kalman_gain = _err_estimate/(_err_estimate + _err_measure);
		_current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
		_err_estimate =  (1.0f - _kalman_gain)*_err_estimate + fabsf(_last_estimate-_current_estimate)*_q;
		_last_estimate=_current_estimate;
	}

	return _current_estimate;
}

void kalmanfilter::set_measurement_error(float mea_e){
	_err_measure = mea_e;
}

void kalmanfilter::set_estimate_error(float est_e){
	_err_estimate = est_e;
}

void kalmanfilter::set_process_noise(float q){
	_q = q;
}

float kalmanfilter::get_gain(void) {
	return _kalman_gain;
}

float kalmanfilter::get_estimate_error(void){
	return _err_estimate;
}







#endif /* ENABLE_COMPONENT_KALMAN_FILER */


