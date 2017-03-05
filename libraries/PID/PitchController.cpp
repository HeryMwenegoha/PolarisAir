#include "PitchController.h"

void AP_PitchController::initialise(AP_Storage *Storage,AP_AHRS *_AP_AHRS)
{
    gains.kp      = &(Storage->ParameterStorage.list.pitch_kp);	
	gains.ki 	  = &(Storage->ParameterStorage.list.pitch_ki);	
	gains.kd 	  = &(Storage->ParameterStorage.list.pitch_kd);	
	gains.tau	  = &(Storage->ParameterStorage.list.pitch_tau);
	gains.rmax 	  = &(Storage->ParameterStorage.list.pitch_rmax);	
	gains.imax    = &(Storage->ParameterStorage.list.pitch_imax);		
	gains.roll_ff = &(Storage->ParameterStorage.list.PTCH2SRV_RLL);
	//gains.max_aux = &(Storage->ParameterStorage.list.max_pitch_aux);	
	//gains.min_aux = &(Storage->ParameterStorage.list.min_pitch_aux);
	
	_ahrs = _AP_AHRS;
}

float AP_PitchController::_turn_coordination()
{
	float gravity_mss = 9.81;
    float _pitch_rad  = _ahrs->pitch;
	float bank  	  = _ahrs->roll;
	bool inverted 	  = false;
	float _roll_ff 	  = *(gains.roll_ff);
	float _rate_out;
	
	if(fabs(bank) < radiansf(90))
	{
		bank  = constrain_float(bank,-radiansf(80), radiansf(80)); // constrained to +/- 80 degrees
	}
	else
	{ 
		inverted = true;
		if(bank > 0.0f)
		bank = constrain_float(bank, radiansf(100), radiansf(180));
		else
		bank = constrain_float(bank, -radiansf(180), -radiansf(100));
	}
	
	if(_speed < 3)
	{
		_speed = 3;
	}
	
	if(fabs(_pitch_rad) < radiansf(70))
	{
		_rate_out = cos(_pitch_rad) * fabs(degreesf() * ((gravity_mss/_speed) * tan(bank) * sin(bank))) * _roll_ff;
	}
	else
	{
		_rate_out = 0;
	}
	
	if(inverted)
	{
		_rate_out = -_rate_out;
	}
	
	/*
	Serial.print(" Sp ");	
	Serial.print(_speed);	
	Serial.print(" G ");
	Serial.print(gravity_mss);	
	Serial.print(" Func ");
	Serial.print(ToDeg((gravity_mss/_speed) * tan(bank) * sin(bank)));
	
	Serial.print(" R ");
	Serial.print(_rate_out);
	Serial.print(" Pi ");
	Serial.print(_pitch_rad);
	Serial.print(" Ro ");
	Serial.print(bank);	
	Serial.print(" Rff ");
	Serial.print(_roll_ff);	
	Serial.print(" OTHERS ");
	*/
	
	return _rate_out;
}


float AP_PitchController::_rate_out(float desired_rate, float meas_rate, float spd_scaler)
{
	uint32_t tnow = millis();
	uint32_t dt   = tnow - _last_Msec;
	if(dt > 1000 || _last_Msec == 0)
	{
		dt = 0;
	}
	_last_Msec = tnow;
	
	float delta_time = (float)dt * 0.001f;
	
    float kp         = *(gains.kp);
	float ki         = *(gains.ki);
	float kd         = *(gains.kd);
	float tau   	 = *(gains.tau);
	float rmax  	 = *(gains.rmax);
    float imax       = *(gains.imax);	// -> needs to be 1/3 of total aileron travel
    //float max_aux    = *(gains.max_aux);	
    //float min_aux    = *(gains.min_aux);	
	
	
	float omega_y    = meas_rate; // body picth rate in deg/sec.
	
	// constrain the desired rate from angle error * 1/T
	if(desired_rate > rmax)
	desired_rate = rmax;
	else if(desired_rate < -rmax)
	desired_rate = -rmax;
	
	// add the extra demanded rate from turn coordination stuff	
	desired_rate  += _turn_coordination();
	
	float rate_error = (desired_rate - omega_y) * spd_scaler;
	
	ki 			  = max(ki, 0.15); // just incase I is to low by user, useful for the tecs controller.
	tau 		  = max(tau, 0.1); // just if user selects a very low value will lead to very high frequency response for small angles
	float kp_ff   = max((kp - ki*tau)*tau - kd, 0);
	float ki_rate = ki * tau;	
	if(dt > 0 && ki_rate > 0 && _speed > 3)
	{
		float i_term = rate_error * ki_rate * delta_time * spd_scaler;
		
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
	_pid_info.P = desired_rate * kp_ff * spd_scaler;
	_pid_info.D = rate_error * kd * spd_scaler;
	
	_output = _pid_info.P + _pid_info.I + _pid_info.D;
	
	float _last_output = constrain_float(_output, -45, 45); // constrain to min/max aileron deflection

	//float servo_pwm  = map_float(_last_output, -45, 45, min_aux, max_aux);
	
	return _last_output;	
}

float AP_PitchController::servo_out(float angle_err, float meas_rate, float spd_scaler)
{
	_speed = _ahrs->airspeed_estimate();
	_speed = constrain_float(_speed, 3.0f, 40.0f);
	
	/*
	Serial.print(" Sp2 ");
	Serial.print(_speed);
	Serial.println("   ");
	*/
	
	float _tau 			= *(gains.tau);
	if(_tau < 0.1)
	_tau = 0.1;
	float _desired_rate = angle_err/_tau;
	
	return _rate_out(_desired_rate, meas_rate, spd_scaler);;
}