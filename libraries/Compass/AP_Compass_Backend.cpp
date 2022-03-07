#include "AP_Compass_Backend.h"

AP_Compass_Backend AP_compass_backend;

void  AP_Compass_Backend::register_device(int8_t instance){
	_compass->instance = instance;
	_compass->registered_devices++;
}

void AP_Compass_Backend::update_magnetometer()
{
	if(_compass->_new_data == true){
		//_compass->_raw_mag_adc = _compass->_filtered_adc; // technically this is the LSB value
		_compass->_mag_adc     = _compass->_filtered_mag_adc; // technically this is the LSB value
		_compass->_new_data = false;
	}	
}

void AP_Compass_Backend::filter_raw_sample(const vector3f &_raw_, float dt)
{
	if(_compass->_low_pass_cutoff <= 0)
		_compass->_low_pass_cutoff = 5;
	float _RC  	= 1/(2 * PI * _compass->_low_pass_cutoff);
	float _alpha_  = dt/(dt + _RC);
	_compass->_filtered_mag_adc =_compass->_filtered_mag_adc + (_raw_ - _compass->_filtered_mag_adc) * _alpha_; // filtered
	_compass->_raw_mag_adc  = _raw_; 																// raw adc value
	
	if(_compass->_filtered_mag_adc.is_nan() || _compass->_filtered_mag_adc.is_inf())
		_compass->_filtered_mag_adc.zero();
		
     _compass->_new_data = true;
}



