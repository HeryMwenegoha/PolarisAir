#include "AP_Pitch.h"

void AP_Pitch::initialise(AP_Storage *Storage,AP_AHRS *_AP_AHRS)
{
    gains.kp      = &(Storage->ParameterStorage.list.pitch_kp);	
	gains.ki 	  = &(Storage->ParameterStorage.list.pitch_ki);	
	gains.kd 	  = &(Storage->ParameterStorage.list.pitch_kd);	
	gains.tau	  = &(Storage->ParameterStorage.list.pitch_tau);
	gains.rmax 	  = &(Storage->ParameterStorage.list.pitch_rmax);	
	gains.imax    = &(Storage->ParameterStorage.list.pitch_imax);		
	gains.roll_ff = &(Storage->ParameterStorage.list.PTCH2SRV_RLL);
	gains.max_aux = &(Storage->ParameterStorage.list.max_pitch_aux);	
	gains.min_aux = &(Storage->ParameterStorage.list.min_pitch_aux);
	_ahrs 		  = _AP_AHRS;
}


float AP_Pitch::_rate_out(float desired_rate, float meas_rate)
{
	uint32_t tnow = millis();
	uint32_t dt   = tnow - _last_Msec;
	if(dt > 1000 || _last_Msec == 0)
	{
		dt 	   = 0;
	}
	_last_Msec 		 = tnow;
	
	float delta_time = (float)dt * 0.001f;
	
    float kp         = *(gains.kp);
	float ki         = *(gains.ki);
	float kd         = *(gains.kd);
	float tau   	 = *(gains.tau);
	float rmax  	 = *(gains.rmax);
    float imax       = *(gains.imax);	// -> needs to be 1/3 of total aileron travel
    float max_aux    = *(gains.max_aux);	
    float min_aux    = *(gains.min_aux);	
		
	
	// constrain the desired rate from angle error * 1/T
	if(desired_rate > rmax)
	desired_rate = rmax;
	else if(desired_rate < -rmax)
	desired_rate = -rmax;
	
	
	float omega_y    = meas_rate; 
	float rate_error = (desired_rate - omega_y);
		
	ki 			  = max(ki, 0); 
	tau 		  = max(tau, 0.1); // just if user selects a very low value will lead to very high frequency response for small angles
	float kp_ff   = max((kp - ki*tau)*tau - kd, 0);
	float ki_rate = ki * tau;	
	
	if(dt > 0 && ki_rate > 0)
	{
		float i_term = rate_error * ki_rate * delta_time;
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
	
	_pid_info.I = constrain_float(_pid_info.I, -imax, imax); // should allow to affect 1/3 of the aileron deflection i.e. 15degrees
	_pid_info.P = desired_rate * kp_ff;
	_pid_info.D = rate_error * kd;
	
	_output 		   = _pid_info.P + _pid_info.I + _pid_info.D;
	float _last_output = constrain_float(_output, -45, 45); // constrain to min/max aileron deflection
	
	return _last_output;	
}

float AP_Pitch::servo_out(float angle_err, float meas_rate)
{
	float _tau 			= *(gains.tau);
	_tau 				= max(_tau, 0.1);
	if(_tau < 0.1)
	_tau = 0.1;
	float _desired_rate = angle_err/_tau;	
	
	_desired_rate	    = _desired_rate * 2.0;
	return _rate_out(_desired_rate, meas_rate);
}