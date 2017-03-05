#pragma once
//#include "AP_Parameters.h"
#include "Common.h"
//#include "AHRS.h"
#include "AP_AHRS.h"
#include "PID_INFO.h"
//#include "AP_GPS.h"

class AP_YawController
{
	public:
	AP_YawController(AP_AHRS &ap_ahrs):
	_ahrs(ap_ahrs),
	_last_rate_hp_out(0),
	_last_rate_hp_in(0)
	{}
	float servo_out(float spd_scaler);
	
	private:
	AP_AHRS &_ahrs;
	float rate_out(float scalar);
	pid_info _pid_info;
	uint32_t last_Msec;
	float _output;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
};