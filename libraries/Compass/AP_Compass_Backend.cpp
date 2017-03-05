#include "AP_Compass_Backend.h"

AP_Compass_Backend AP_compass_backend;

void AP_Compass_Backend::update_magnetometer()
{
	if(_compass->_new_data == true){
		_compass->_raw_mag = _compass->_filtered_adc;
		_compass->_new_data = false;
	}	
}

void AP_Compass_Backend::filter_raw_sample(const vector3f &_raw_, float dt)
{
	if(_compass->_low_pass_cutoff <= 0)
		_compass->_low_pass_cutoff = 5;
	float _RC  	= 1/(2 * PI * _compass->_low_pass_cutoff);
	float _alpha_  = dt/(dt + _RC);
	_compass->_filtered_adc =_compass->_filtered_adc + (_raw_ - _compass->_filtered_adc) * _alpha_;
	
	if(_compass->_filtered_adc.is_nan() || _compass->_filtered_adc.is_inf())
		_compass->_filtered_adc.zero();
		
     _compass->_new_data = true;
}



