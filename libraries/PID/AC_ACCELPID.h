#pragma once
#include "AP_Parameters.h"
#include "PID_INFO.h"

class AC_ACCELPID
{
	public:
	AC_ACCELPID():
	_input(0),
	_derivative(0),
	_dt(20),
	_last_update_us(0),
	//_params(params),
	FilterFrequency(5),
	_output(0),
	kp     (0),
	ki	   (0),
	kd	   (0),
	imax   (5)
	{
			
	}
	
	float get_pid(float );
	
	float set_pid(float _kp, float _ki, float _kd, float _imax){
	 kp     = _kp;
	 ki     = _ki;
     kd     = _kd;
     imax   = _imax;
	}
    void  reset();
	
	private:
	float _input;
	float _derivative;
	float FilterFrequency;
	float _output;
	float _dt;
	
	uint64_t _last_update_us;
	pid_info _pid_info;
	//_gains   gains;
	
	protected:
	//AP_Storage *_params;	
	float kp;
	float ki;
    float kd;
    float imax;
};