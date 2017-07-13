#include "SteerController.h"

void AP_SteerController::initialise(AP_Storage *_AP, AP_GPS * _AP_GPS)
{
	gains.kp   = &(_AP->ParameterStorage.list.steer_kp);
	gains.ki   = &(_AP->ParameterStorage.list.steer_ki);
	gains.kd   = &(_AP->ParameterStorage.list.steer_kd);
	gains.tau  = &(_AP->ParameterStorage.list.steer_tau);
	gains.rmax = &(_AP->ParameterStorage.list.steer_rmax);
	gains.imax = &(_AP->ParameterStorage.list.steer_imax);
	
	_gps	= _AP_GPS;
}

// returns a servo deflection in the range of +/- 45 degrees
// angle_err in deg
// meas_rate in deg/sec
float AP_SteerController::servo_out(float angle_err, float meas_rate)
{
	float _tau = *(gains.tau);
	float _desired_rate = angle_err/_tau; // deg/sec
	return rate_out(_desired_rate, meas_rate);
}

float AP_SteerController::rate_out(float desired_rate, float meas_rate)
{
	uint32_t t_now = millis();
	uint32_t dt    = t_now - last_Msec;
	if(dt > 1000 || last_Msec == 0)
	{
		dt = 0;
	}
	float delta_time = (float)dt * 0.001f;
	last_Msec = t_now;
	
	float kp  	   = *(gains.kp);
	float ki  	   = *(gains.ki);
	float kd  	   = *(gains.kd);
	float tau      = *(gains.tau);
	float rmax     = *(gains.rmax);
	float imax     = *(gains.imax);
	float minspeed = 1.0f; // start integrating after 1m/s ground speed	
	float speed    = _gps->groundspeed();
	speed 		   = max(speed, 0.75);
	float scalar   = 2.0/speed;
	float rate_error = (desired_rate - meas_rate) * scalar;
	
	tau = max(tau, 0.1);
	ki  = max(ki, 0);
	float kp_ff   = max((kp - ki*tau)*tau - kd,0)*45.0f;
	float ki_rate = ki * tau * 45.0f;
	
	
	if(dt > 0 && ki_rate > 0 && speed >= minspeed)
	{
		float i_term = rate_error * delta_time * ki_rate * scalar;
		if(_output > 45)
		{
			i_term = min(i_term, 0);
		}
		else if(_output < -45)
		{
			i_term = max(i_term, 0);
		}
		_pid_info.I += i_term;
	}
	else
	{
		_pid_info.I = 0;
	}
	
	_pid_info.I = constrain_float(_pid_info.I, -imax, imax);
	_pid_info.P = radiansf(desired_rate) * kp_ff * scalar;
	_pid_info.D = rate_error * kd * 4.0f;
	
	_output  =  _pid_info.P + _pid_info.I + _pid_info.D;
	
	return constrain_float(_output, -45, 45);
}

