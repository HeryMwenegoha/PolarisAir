#include "RollController.h"
void AP_RollController::initialise(AP_Storage *_AP,  AP_AHRS *_AP_AHRS)
{
	gains.kp	= &(_AP->ParameterStorage.list.roll_kp);	
	gains.ki	= &(_AP->ParameterStorage.list.roll_ki);	
	gains.kd	= &(_AP->ParameterStorage.list.roll_kd);	
	gains.tau	= &(_AP->ParameterStorage.list.roll_tau);
	gains.rmax	= (float *)&(_AP->ParameterStorage.list.roll_rmax);	
	gains.imax	= (float *)&(_AP->ParameterStorage.list.roll_imax);			
	_ahrs		= _AP_AHRS;
}

float AP_RollController::_rate_out(float desired_rate, float meas_rate, float spd_scaler)
{		
	uint32_t t_now = millis();
	uint32_t dt    = t_now - _last_Msec;
	if(_last_Msec == 0 || dt > 1000)
	{
		dt = 0;
	}	
	_last_Msec       = t_now;
	float delta_time = (float)dt * 0.001f; 
	
	float KP         = *(gains.kp);
	float KI         = *(gains.ki);
	float KD         = *(gains.kd);
	float TAU   	 = *(gains.tau);
	float RMAX  	 = *(gains.rmax);
    float IMAX       = *(gains.imax);	// -> needs to be 1/3 of total aileron travel	
   
    float ki_rate    = KI* TAU; 
	float EAS2TAS    = 1;
	float kp_ff      = max((KP - KI * TAU)*TAU - KD, 0)/EAS2TAS;

	// desired rate limiter
	if(desired_rate > RMAX)
		desired_rate = RMAX;        // constrained to max deg/sec && min deg/sec
    else if(desired_rate < -RMAX)
		desired_rate = -RMAX;
	
	// body rate..
	float _omega_x   = meas_rate;	// deg/sec
	
	float _rate_error = (desired_rate - meas_rate) * spd_scaler;
	
	if(dt > 0  && ki_rate > 0 && _speed > 3)
	{
		float I_Term = _rate_error * ki_rate * delta_time * spd_scaler;
		
		if(_output > 45)
		{
			I_Term = min(I_Term, 0);
		}
		else if(_output < -45)
		{
			I_Term = max(I_Term, 0);
		}		
		_pid_info.I += I_Term;
	}
	else
	{
		_pid_info.I = 0; // mostly a reset i.e. going from manual to stabilise were dt > 1000 and therefore dt = 0
	}
	
	_pid_info.I = constrain_float(_pid_info.I, -IMAX, IMAX);	
	_pid_info.P = desired_rate * kp_ff * spd_scaler;
	_pid_info.D = _rate_error  * KD * spd_scaler;
	
	_output 	= _pid_info.P + _pid_info.I + _pid_info.D;
	
	float _last_output = constrain_float(_output, -45, 45); // constrain to min/max aileron deflection
		
	return _last_output;
}

// we want the demanded aileron deflection mapped to +/- 500 PWM units
// angle_err  in deg
// meas_rate  in deg/sec
// spd_scaler in scalar quantity = ref_speed/airspeed
float AP_RollController::servo_out(float angle_err, float meas_rate, float spd_scaler)
{	
	_speed = _ahrs->airspeed_estimate();
	_speed = constrain_float(_speed, 3.0f, 40.0f);
	float tau        = *(gains.tau);
	
	float desired_rate = angle_err / tau;
		
	return _rate_out(desired_rate, meas_rate, spd_scaler);	
}