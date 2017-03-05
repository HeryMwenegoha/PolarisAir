#include "AP_HgtFilter.h"
void AP_HgtFilter::update(){
		// Get altitude estimate
	uint64_t tnow = micros();
	float DT      = (tnow - _update_last_usec) * 1e-6f;
	_update_last_usec  = tnow;
  
	if(DT > 1.0f)
	{
	  _integ3_state    = 0.0f;
	  _climb_rate      = 0.0f;
	  _integ1_state    = 0.0f;
	  DT               = 0.02f; // 50Hz
	}   	
	
	if(_ahrs.healthy() == false){
		return;
	}
	
	float baro_alt 	   = _ahrs.baro().get_altitude();
	
	vector3f accel_bf  = vector3f(_ahrs.acc.x, _ahrs.acc.y, _ahrs.acc.z);	// body frame accelerations
	vector3f accel_ef  = _ahrs.dcm() * accel_bf;			// accelerations in the earth frame
	float hgt_ddot_mea = -(accel_ef.z + gravity); 			        // NED positive

	// Filter calculations
	_hgtCompFiltOmega = 0.27;//_params.ParameterStorage.list.PowerModule_Gain;
	
	float omega2       = _hgtCompFiltOmega * _hgtCompFiltOmega;
	float hgt_err      = baro_alt - _integ3_state;
	float integ1_input = hgt_err * omega2  * _hgtCompFiltOmega;
	
	_integ1_state      = _integ1_state     + integ1_input * DT;

	float integ2_input = hgt_ddot_mea + _integ1_state + hgt_err * omega2 * 3.0f;
	_climb_rate        = _climb_rate  + integ2_input  * DT;

	float integ3_input = _climb_rate + hgt_err * _hgtCompFiltOmega * 3.0f;
	
	// There's no need for this but just incase you know..
	if(DT > 1.0f)
	{
	  _integ3_state = baro_alt;
	}
	else
	{
	   _integ3_state = _integ3_state + integ3_input* DT; 
	}
}


float AP_HgtFilter::height(){
	return _integ3_state;
}

float AP_HgtFilter::altitude(){
	return _integ3_state;
}


float AP_HgtFilter::climbrate(){
	return _climb_rate;
}
