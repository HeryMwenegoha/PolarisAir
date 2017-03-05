#pragma once
#include "AP_Parameters.h"
#include "Common.h"
#include "AHRS.h"
#include "PID_INFO.h"
#include "AP_AHRS.h"


class AP_Pitch
{
	public: 
	AP_Pitch(){
	};
	void  initialise(AP_Storage *, AP_AHRS *);
	float servo_out(float angle_err, float meas_rate);
	
	private:
	float _rate_out(float desired_rate, float meas_rate);	
	
	pid_info _pid_info;
	_Pgains gains;
	
	protected:
	float    _output;
	uint32_t _last_Msec;		
	AP_AHRS  *_ahrs;
};