#pragma once
#include "AP_Parameters.h"
#include "Common.h"
#include "AHRS.h"
#include "PID_INFO.h"
#include "AP_GPS.h"

class AP_Yaw
{
	public:
	AP_Yaw(){}
	void initialise(AP_Storage *_AP);
	float servo_out(float angle_err, float meas_rate);
	
	private:
	float rate_out(float desired_rate, float meas_rate);
	_gains gains;
	pid_info _pid_info;
	uint32_t last_Msec;
	float _output;
};