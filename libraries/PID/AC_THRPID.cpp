#include "AC_THRPID.h"
#include "Common.h"

// input = desired_rate - measured_rate
float AC_THRPID::get_pid(float input){
	
	// update timer
	uint64_t now    = micros();
	_dt             = (now - _last_update_us) * 1e-6f;	
	if(_last_update_us == 0 || _dt >= 1.0){
		_derivative = 0;
		_dt         = 0;
		_pid_info.I = 0;
	}
	_last_update_us = now;
	
	
	// update filter
	float RC    = 1/(2 * PI * FilterFrequency);
	float alpha = _dt/(_dt + RC);
	
	
	// update derivative
	if(_dt > 0.0f)
	{
		float derivative = (input - _input)/_dt;
		_derivative      =  _derivative + alpha * (derivative - _derivative);
	}
		
	// update input after derivative calculations
	_input 		= input;
	
	
	// Integrator
	if(ki > 0 && _dt > 0){
		float i_term = _input * _dt * ki;
		if(_output      > 1500)
		{
			i_term = min(i_term, 0);
		}
		else if(_output < -500)
		{
			i_term = max(i_term, 0);
		}
		_pid_info.I += i_term;
	}
	else
	{
		_pid_info.I = 0;
	}	
	
	// get PIDs
	_pid_info.P = kp * _input;
	_pid_info.I = constrain_float(_pid_info.I, -imax, imax); 
	_pid_info.D = kd * _derivative;
	
	_output     = _pid_info.P  + _pid_info.I + _pid_info.D;
	
	return constrain_float(_output, -500, 1500);
	
}

void  AC_THRPID::reset(){
	_pid_info.I = 0;
	_derivative = 0;
}