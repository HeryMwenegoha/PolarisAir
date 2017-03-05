#include "AP_INS_Backend.h"
AP_INS_Backend AP_ins_backend;

AP_INS_Backend::AP_INS_Backend():
	_imu(&AP_ins)
{

}

void AP_INS_Backend::update_gyro(uint8_t instance)
{	
	if(_imu->_new_gyro_data[instance]){
		_imu->_gyro_[instance] = _imu->_filtered_gyro[instance];
		_imu->_new_gyro_data[instance] = false;
	}	
}

void AP_INS_Backend::update_accel(uint8_t instance)
{	
	if(_imu->_new_accel_data[instance]){
		_imu->_accel_[instance] = _imu->_filtered_accel[instance];
		_imu->_new_accel_data[instance] = false;
	}
}

void AP_INS_Backend::filter_raw_sample_gyro(uint8_t instance , const vector3f &_raw_gyro, float dt)
{
	if(_imu->_low_pass_cutoff_gyro <= 0)
		_imu->_low_pass_cutoff_gyro = 5;
	float _RC_gyro  		= 1/(2 * PI * _imu->_low_pass_cutoff_gyro);
	float _alpha_gyro  		= dt/(dt + _RC_gyro);
	_imu->_filtered_gyro[instance] = _imu->_filtered_gyro[instance] + (_raw_gyro - _imu->_filtered_gyro[instance]) * _alpha_gyro;
	
	if(_imu->_filtered_gyro[instance].is_nan() || _imu->_filtered_gyro[instance].is_inf())
		_imu->_filtered_gyro[instance].zero();
		
     _imu->_new_gyro_data[instance] = true;
}

void AP_INS_Backend::filter_raw_sample_accel(uint8_t instance, const vector3f &_raw_accel, float dt)
{
	if(_imu->_low_pass_cutoff_accel <= 0)
		_imu->_low_pass_cutoff_accel = 5;
	float _RC_accel 		  = 1/(2 * PI * _imu->_low_pass_cutoff_accel);
	float _alpha_accel 			  = dt/(dt + _RC_accel);
	_imu->_filtered_accel[instance] = _imu->_filtered_accel[instance] + (_raw_accel - _imu->_filtered_accel[instance]) * _alpha_accel;
	
	if(_imu->_filtered_accel[instance].is_nan() || _imu->_filtered_accel[instance].is_inf())
		_imu->_filtered_accel[instance].zero();
		
	_imu->_new_accel_data[instance] = true;
}