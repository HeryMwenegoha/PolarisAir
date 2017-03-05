#pragma once
#include "AP_Parameters.h"
#include "Common.h"
#include "AHRS.h"
#include "PID_INFO.h"

#include "AP_AHRS.h"
#include "AP_Airspeed.h"

// need to add a stall prevent with airspeed calculations
class AP_PitchController
{
	public: 
	AP_PitchController(){
	};
	void initialise(AP_Storage *, AP_AHRS *);
	float servo_out(float angle_err, float meas_rate, float spd_scaler);
	
	private:
	float _rate_out(float desired_rate, float meas_rate, float spd_scaler);	
	float _turn_coordination();
	
	pid_info _pid_info;
	_Pgains gains;
	
	protected:
	float _output;
	float _speed;
	uint32_t _last_Msec;
		
	AP_AHRS *_ahrs;
};