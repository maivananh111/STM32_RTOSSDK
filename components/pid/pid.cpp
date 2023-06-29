/*
 * pid.cpp
 *
 *  Created on: May 11, 2023
 *      Author: anh
 */
#include "component_config.h"

#if ENABLE_COMPONENT_PID

#include "pid/pid.h"

#include "periph/systick.h"


pid::pid(void){}

void pid::init(pid_param_t *param){
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

void pid::set_param(pid_param_t *param){
	float ratio = (float)param->sample_time / (float)_param->sample_time;
	_param = param;

	_param->ki *= ratio;
	_param->kd /= ratio;
}

pid_param_t *pid::get_param(void){
	return _param;
}

void pid::set_point(float point){
	_setp = point;
}

float pid::calculate(float input){
	_time = get_tick();
	_dtime = (_time - _ltime);

	if(_dtime >= _param->sample_time){
	    _err = _setp - input;
	    _derr = _err - _lerr;

	    _P  = _param->kp * _err;
		_I += _param->ki * (_err * _dtime);
		_D  = _param->kd * (_derr / _dtime);

		_out = _P + _I + _P;

		if(_out > _param->max_output) {
			_out = _param->max_output;
			_I -= _err * _dtime; /** anti-windup */
		}
		if(_out < _param->min_output) {
			_out = _param->min_output;
			_I -= _err * _dtime; /** anti-windup */
		}

		_lerr = _err;
		_ltime = _time;
	}

	return _out;
}

#endif /* ENABLE_COMPONENT_PID */
