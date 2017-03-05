#include "YawController.h"
// returns a servo deflection in the range of +/- 45 degrees
float AP_YawController::servo_out(float spd_scaler)
{
	return rate_out(spd_scaler);
}



float AP_YawController::rate_out(float scalar)
{
	uint32_t t_now = millis();
	uint32_t dt    = t_now - last_Msec;
	if(dt > 1000 || last_Msec == 0)
	{
		dt = 0;
	}
	float delta_time = (float)dt * 0.001f;
	last_Msec = t_now;
	
	float aspeed;
	float rate_offset;
	float bank_angle = _ahrs.roll;
	
	if(fabs(bank_angle) < 1.5707964f){
		bank_angle = constrain_float(bank_angle, -1.3962634f,1.3962634f);
	}
	
	aspeed = _ahrs.airspeed_estimate();
	float aspmin = 5;
	
	// rate offset rad/sec
	rate_offset = (9.8065/max(aspeed, float(aspmin))) * tanf(bank_angle) * cosf(bank_angle) * 1.0f;
	
	// body rate (rad/sec)
	float omega_z = _ahrs.yawrate;
	
	// sideslip acceleration in mss
	float accel_y = _ahrs.acc.y;
	
	// Get relative rate for a coordinated turn
	float rate_hp_in = ToDeg(omega_z - rate_offset);
	
	// apply high pass filter to remove bias due to rate offset
	// cut off frequency 0.2rad/sec
	float rate_hp_out = 0.9960080f * _last_rate_hp_out + (rate_hp_in - _last_rate_hp_in);
	_last_rate_hp_out = rate_hp_out;
	_last_rate_hp_in = rate_hp_in;
	
	
	// Only Yaw Damper to damp ruddermix
	// I have neglected the sideslip controller
	float _K_D  = 0.5;
	_pid_info.D = _K_D * (-rate_hp_out) * scalar * scalar;
	_output     =  _pid_info.D;
	
	return constrain_float(_output, -45, 45);
}

