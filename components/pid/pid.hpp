/*
 * pid.h
 *
 *  Created on: May 11, 2023
 *      Author: anh
 */

#ifndef COMPONENTS_PID_H_
#define COMPONENTS_PID_H_

#include "component_config.h"
#if ENABLE_COMPONENT_PID

#define PID_DEBUG 0

#include "sdkconfig.h"
#include st_header
#include "stdio.h"

#include "periph/systick.h"

#if PID_DEBUG
#include "system/log.h"
#endif /* PID_DEBUG */

typedef enum{
	PID_NORMAL,
	PID_REVERSE,
} pid_direction_t;

typedef struct {
	pid_direction_t direction = PID_NORMAL;
	float kp = 0.0;
	float ki = 0.0;
	float kd = 0.0;
	uint32_t max_output = 0;
	uint32_t min_output = 0;
	uint32_t precision = 10;
	uint32_t sample_time = 10U; // 10ms.
} pid_param_t;

template < class pid_var_type>
class pid{
	public:
		pid(void);
		void init(pid_param_t *param);

		void set_param(pid_param_t *param);
		pid_param_t *get_param(void);
		void set_point(pid_var_type point);

		pid_var_type calculate(pid_var_type input);

	private:
		pid_param_t *_param = NULL;
		pid_var_type _P = 0, _I = 0, _D = 0;
		pid_var_type _err = 0, _lerr = 0;
		int32_t _derr;
		uint32_t _time = 0U, _ltime = 0U, _dtime;
		pid_var_type _setp = 0;
		pid_var_type _out = 0;
};




template < class pid_var_type>
pid<pid_var_type>::pid(void){}

template < class pid_var_type>
void pid<pid_var_type>::init(pid_param_t *param){
	float ratio = (float)param->sample_time / 1000U;
	_param = param;

	if(_param->max_output <= _param->min_output){
		return;
	}

	_param->ki *= ratio;
	_param->kd /= ratio;

	if(_param->direction == PID_REVERSE){
		_param->kp = -1 * _param->kp;
		_param->ki = -1 * _param->ki;
		_param->kd = -1 * _param->kd;
	}

	_time = get_tick();
}

template < class pid_var_type>
void pid<pid_var_type>::set_param(pid_param_t *param){
	float ratio = (float)param->sample_time / (float)_param->sample_time;
	_param = param;

	_param->ki *= ratio;
	_param->kd /= ratio;

	if(_param->direction == PID_REVERSE){
		_param->kp = -1 * _param->kp;
		_param->ki = -1 * _param->ki;
		_param->kd = -1 * _param->kd;
	}
}

template < class pid_var_type>
pid_param_t *pid<pid_var_type>::get_param(void){
	return _param;
}

template < class pid_var_type>
void pid<pid_var_type>::set_point(pid_var_type point){
	_setp = point;
}

template < class pid_var_type>
pid_var_type pid<pid_var_type>::calculate(pid_var_type input){
	_time = get_tick();
	_dtime = (_time - _ltime);

	if(_dtime >= _param->sample_time){
	    _err = _setp - input;
	    _derr = _err - _lerr;

	    _P  = _param->kp * _err;
	    if(_err > -1*(int32_t)_param->precision && _err < (int32_t)_param->precision){
	    	_I += _param->ki * (_err * (int32_t)((int32_t)_dtime/_param->sample_time));
	    }
	    _D  = _param->kd * (int32_t)((int32_t)_derr / (int32_t)((int32_t)_dtime/_param->sample_time));

		_out = (int32_t)_P + (int32_t)_I + (int32_t)_D;

		if(_out >= (pid_var_type)_param->max_output) {
			_out = (pid_var_type)_param->max_output;
		}
		if(_out <= (pid_var_type)_param->min_output) {
			_out = (pid_var_type)_param->min_output;
		}

#if PID_DEBUG
		LOG_DEBUG("PID",  "set: %5d, input: %5d, error: %5d, dtime: %5d, derr:  %5d, P = %5d, I = %4d, D = %5d, _out = %5d",
				_setp, input, _err, _dtime, _derr, _P, _I, _D, _out);
#endif /* PID_DEBUG */

		_lerr = _err;
		_ltime = _time;
	}

	return _out;
}

#endif /* ENABLE_COMPONENT_PID */

#endif /* COMPONENTS_PID_H_ */
