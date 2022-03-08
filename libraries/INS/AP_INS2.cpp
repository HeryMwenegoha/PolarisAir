#include "AP_INS2.h"

void AP_INS2::add_backend(AP_INS_Backend *_ptr)
{
	if(!_ptr){
		_backends[instance++] = _ptr;
	}
}

bool AP_INS2::calibrate()
{
	// 1st run
	if(	_calibrate_run == 0)
	{
		for(int instance = 0; instance < 3; instance++)
		{
			_gyro_offset[instance]  =  _gyro_offset[instance] * 0.7f +  _gyro_[instance] * 0.3f;  
			_accel_offset[instance] = _accel_offset[instance] * 0.7f +  _accel_[instance] * 0.3f;			
		}			
		_calibrate_count++;		
		if(_calibrate_count == 50)
		{
			for(int i = 0; i < 3; i++)
			{
				_offset_initial[i] = _gyro_offset[i];
			}
			_calibrate_run++;
			_calibrate_count = 0;
			_gyro_offset[0].zero();
			_gyro_offset[1].zero();
			_gyro_offset[2].zero();
		}
		return false;
	}else if(_calibrate_run == 1){		
		// 2nd run
		for(int instance = 0; instance < 3; instance++)
		{
			_gyro_offset[instance]  =  _gyro_offset[instance] * 0.7f +  _gyro_[instance] * 0.3f;  
			_accel_offset[instance] = _accel_offset[instance] * 0.7f +  _accel_[instance] * 0.3f;
		}	
		_calibrate_count++;	
		if(_calibrate_count < 50){
			return false;
		}else if(_calibrate_count >= 50)
		{
			vector3f _offset_final[3];
			for(int i = 0; i < 3; i++)
			{
				_offset_final[i] = _gyro_offset[i];
			}
			
			_calibrate_run++;
			_calibrate_count = 0;
			_gyro_offset[0].zero();
			_gyro_offset[1].zero();
			_gyro_offset[2].zero();
			
			for(int i = 0; i<3; i++)
			{
				
				Serial.print(F("GI:	"));
				Serial.print(_offset_initial[i].x);
				Serial.print(F("	"));
				Serial.print(_offset_initial[i].y);
				Serial.print(F("	"));
				Serial.print(_offset_initial[i].z);
				Serial.print(F("	"));
				
				Serial.print(F("GF:	"));
				Serial.print(_offset_final[i].x);
				Serial.print(F("	"));
				Serial.print(_offset_final[i].y);
				Serial.print(F("	"));
				Serial.print(_offset_final[i].z);
				Serial.println(F("	"));
			
			}
			
			vector3f _diff[3];
			vector3f _sum[3];
			for(int i = 0; i<3; i++)
			{				
				_diff[i] = _offset_final[i] - _offset_initial[i];
				_sum[i]  = (_offset_final[i] + _offset_initial[i]) * 0.5f;
				
				if(fabs(_diff[i].x) < 800  &&
				   fabs(_diff[i].y) < 800  &&
				   fabs(_diff[i].z) < 800	&&
				   fabs(_sum[i].x ) > 0		&&
				   fabs(_sum[i].y)  > 0		&&
				   fabs(_sum[i].z)  > 0
				   ) {					
						_gyro_offset[i] =  _sum[i];	
						_gyro_health_count[i] = true;
						
						/*
						PRINT HEALTHY GYRO OFFSETS
						Serial.print(F("Gyro:  "));
						_gyro_offset[i].printV();
						*/						
				 }
				 				
				if(!_accel_offset[i].is_zero()){
						_accel_health_count[i] =true;
						_accel_offset[i].zero();
						/*
						PRINT HEALTHY ACCEL OFFSETS
						Serial.print(F("Accel:  "));
						_accel_offset[i].printV();
						*/
				}
			}
			
			if( (_accel_health_count[0] || _accel_health_count[1] ||_accel_health_count[2]) &&
				(_gyro_health_count[0]  || _gyro_health_count[1]  || _gyro_health_count[2])){
				_have_ins = true;
			}else{
				Serial.println(F("AP_INS::No	Healthy	Sensors	Detected"));
			}
			return true;
		}
	}
}


void  AP_INS2::update()
{	
	if(_hil_mode == true)
		return;	
	
	if(_calibrated == false){
		_calibrated = calibrate();
		return;
	}
	
	_accel_offset[L3Gd20] = vector3f(
	_parameters->ParameterStorage.list.Accel2_offsetX,
	_parameters->ParameterStorage.list.Accel2_offsetY,
	_parameters->ParameterStorage.list.Accel2_offsetZ); 
	
    _accel_offset[MPU6000] = vector3f(
	_parameters->ParameterStorage.list.Accel2_offsetX,
	_parameters->ParameterStorage.list.Accel2_offsetY,
	_parameters->ParameterStorage.list.Accel2_offsetZ); 

	_accel_scale = vector3f(
	_parameters->ParameterStorage.list.Accel2_lsbX,
	_parameters->ParameterStorage.list.Accel2_lsbY,
	_parameters->ParameterStorage.list.Accel2_lsbZ);	
	

	
	for(int i = 0; i < 3; i++)
	{
		_raw_accel[i]  = _accel_[i];  			// ADC LSB
		_raw_gyro[i]   = _gyro_[i];   			// ADC LSB
	}

	_gyro[L3Gd20]  = (_raw_gyro[L3Gd20]   - _gyro_offset[L3Gd20])  * radiansf(0.0175);		// radians per second 500DPS
	_gyro[MPU6000] = (_raw_gyro[MPU6000]  - _gyro_offset[MPU6000]) * radiansf(0.0152672f);	// radians per second 500DPS
	_gyro[L3Gd20H] = (_raw_gyro[L3Gd20H]  - _gyro_offset[L3Gd20H]) * radiansf(0.0175);
			
	_accel[L3Gd20].x  = (_raw_accel[L3Gd20].x  - _accel_offset[L3Gd20].x)  * (9.81/_accel_scale.x);
	_accel[L3Gd20].y  = (_raw_accel[L3Gd20].y  - _accel_offset[L3Gd20].y)  * (9.81/_accel_scale.y);
	_accel[L3Gd20].z  = (_raw_accel[L3Gd20].z  - _accel_offset[L3Gd20].z)  * (9.81/_accel_scale.z);		
	
	_accel[MPU6000].x = (_raw_accel[MPU6000].x - _accel_offset[MPU6000].x) * (9.81/_accel_scale.x);
	_accel[MPU6000].y = (_raw_accel[MPU6000].y - _accel_offset[MPU6000].y) * (9.81/_accel_scale.y);
	_accel[MPU6000].z = (_raw_accel[MPU6000].z - _accel_offset[MPU6000].z) * (9.81/_accel_scale.z);
	
		
	// Grab values from Backend and apply all the neccessary scaling and whatnots
	if(((_accel[MPU6000].is_zero() || _accel[MPU6000].is_nan()) && (_gyro[MPU6000].is_zero() || _gyro[MPU6000].is_nan())) &&
	    ((_accel[L3Gd20].is_zero()  || _accel[L3Gd20].is_nan())  && (_gyro[L3Gd20].is_zero() || _gyro[L3Gd20].is_nan()))
		){
	}else{
		_last_update_msec = millis();
	}
	
	/*
	Serial.print(_accel.x);
	Serial.print("	");
	Serial.print(_accel.y);
	Serial.print("	");
	Serial.println(_accel.z);	
	*/
}

vector3f* AP_INS2::gyro()
{
	return _gyro;
}

vector3f* AP_INS2::accel()
{
	return _accel;
}

vector3f* AP_INS2::raw_gyro()
{
	return _raw_gyro;
}

vector3f* AP_INS2::raw_accel()
{
	return _raw_accel;
}


void     AP_INS2::setHil(const float &_rr, const float &_pr, const float &_yr, const float &_xacc, const float &_yacc, const float &_zacc)
{
	_hil_mode 		       = true;
	_have_ins 			   = true;
	_accel_health_count[0] = _accel_health_count[1] = _accel_health_count[2] = true;
	_gyro_health_count[0]  = _gyro_health_count[1] = _gyro_health_count[2] = true;
	_last_update_msec      = millis();
	
	/* Radians Per Second*/
	_gyro[0] = vector3f(_rr, _pr, _yr); 
	_gyro[1] = vector3f(_rr, _pr, _yr);
	_gyro[2] = vector3f(_rr, _pr, _yr);
	
	
	/* Metres Per Second Squared*/
	_accel[0] = vector3f(_xacc, _yacc, _zacc);	
	_accel[1] = vector3f(_xacc, _yacc, _zacc);	
	_accel[2] = vector3f(_xacc, _yacc, _zacc);	
	
	
	// Need to change this for raw values| Not very necessary though
	_raw_gyro[L3Gd20]   = _gyro[L3Gd20]    *  (degreesf() * (1/0.0175));  		  // LSB
	_raw_gyro[MPU6000]  = _gyro[MPU6000]   *  (degreesf() * 65.5f);   			  // LSB
	_raw_accel[L3Gd20]  = _accel[L3Gd20]   * (8192.0f/9.81f); 	
	_raw_accel[MPU6000] = _accel[MPU6000]  * (8192.0f/9.81f); 		  			  // LSB
}

bool  	 AP_INS2::have_ins()
{
	return _have_ins;
}

uint32_t AP_INS2::last_update_msec()
{
	return _last_update_msec;
}

bool AP_INS2::healthy()
{
	return ((millis() - _last_update_msec) < 1100) && _have_ins; // delay in sending ins message to the gcs might cause raising this error
}
