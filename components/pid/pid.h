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

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "stdio.h"


typedef enum{
	PID_NORMAL,
	PID_REVERSE,
} pid_direction_t;

typedef struct {
	pid_direction_t direction = PID_NORMAL;
	float kp = 0.0;
	float ki = 0.0;
	float kd = 0.0;
	float max_output = 0;
	float min_output = 0;
	uint32_t sample_time = 10U; // 10ms.
} pid_param_t;

class pid{
	public:
		pid(void);
		void init(pid_param_t *param);

		void set_param(pid_param_t *param);
		pid_param_t *get_param(void);
		void set_point(float point);

		float calculate(float input);

	private:
		pid_param_t *_param = NULL;
		float _P = 0.0, _I = 0.0, _D = 0.0;
		float _err = 0.0, _lerr = 0.0, _derr;
		uint32_t _time = 0U, _ltime = 0U, _dtime;
		float _setp = 0.0, _out = 0.0;
};




#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_PID */

#endif /* COMPONENTS_PID_H_ */
