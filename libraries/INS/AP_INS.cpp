#include "AP_INS.h"

#include "AP_INS_Backend2.h" // Since This class is forward declared in the h file i am including it here to prevent compiler errors

AP_INS      AP_ins;

void AP_INS::add_backend(AP_INS_Backend2 *_ptr)
{
	if(_ptr != NULL){
		_backends[instances++] = _ptr;
		Serial.println(F("Backend added"));
		//Serial.println(_ptr->LSB2DPS,5);
		//Serial.println(_backends[0]->LSB2DPS,5);
	}
}

bool AP_INS::calibrate()
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
				Serial.println(F("AP_INS:: Sensors	Calibrated"));
				_have_ins = true;
			}else{
				Serial.println(F("AP_INS::No	Healthy	Sensors	Detected"));
			}
			return true;
		}
	}
}


void  AP_INS::update()
{	
	if(_hil_mode == true)
		return;	

	if(_calibrated == false){
		_calibrated = calibrate();
		return;
	}
	
	_accel_offset[0] = vector3f(
	_parameters->ParameterStorage.list.Accel2_offsetX,
	_parameters->ParameterStorage.list.Accel2_offsetY,
	_parameters->ParameterStorage.list.Accel2_offsetZ); // 0

	_accel_scale = vector3f(
	_parameters->ParameterStorage.list.Accel2_lsbX,
	_parameters->ParameterStorage.list.Accel2_lsbY,
	_parameters->ParameterStorage.list.Accel2_lsbZ);	// 8192
	
	
	//_accel_offset[0].print();
	//_accel_scale.print();
	
	// Select Gyroscope and Accelerometer Scaler
	float g_scaler = 1;
	float a_scaler = 1;
	for(uint8_t i=0; i<instances; i++)
	{
		switch(_backends[i]->CHAR)
		{
			case 'G':
				g_scaler = radiansf(_backends[i]->SCALER);    // LSB2DPS
				break;

			case 'A':
				if(_backends[i]->TYPE == FXOS8700)
					//a_scaler = 8192*_backends[i]->SCALER;     // LSB2G's
				break;
		}
	}
	
	for(int i = 0; i < 3; i++)
	{
		_raw_accel[i]  = _accel_[i];  			// ADC LSB
		_raw_gyro[i]   = _gyro_[i];   			// ADC LSB
	}
	
	_gyro[0]     = (_raw_gyro[0]     - _gyro_offset[0])  * g_scaler;		
			
	_accel[0].x  = (_raw_accel[0].x  - _accel_offset[0].x)  * (1/_accel_scale.x) * a_scaler * 9.81;
	_accel[0].y  = (_raw_accel[0].y  - _accel_offset[0].y)  * (1/_accel_scale.y) * a_scaler * 9.81;
	_accel[0].z  = (_raw_accel[0].z  - _accel_offset[0].z)  * (1/_accel_scale.z) * a_scaler * 9.81;		
	
	
		
	// Grab values from Backend and apply all the neccessary scaling and whatnots
	if(((_accel[MPU6000].is_zero() || _accel[MPU6000].is_nan()) && (_gyro[MPU6000].is_zero() || _gyro[MPU6000].is_nan())) &&
	    ((_accel[L3Gd20].is_zero()  || _accel[L3Gd20].is_nan())  && (_gyro[L3Gd20].is_zero() || _gyro[L3Gd20].is_nan()))
		){
	}else{
		_last_update_msec = millis();
	}
	
	//_accel[0].print();
	
	/*
	Serial.print(_accel.x);
	Serial.print("	");
	Serial.print(_accel.y);
	Serial.print("	");
	Serial.println(_accel.z);	
	*/
}

vector3f* AP_INS::gyro()
{
	return _gyro;
}

vector3f* AP_INS::accel()
{
	return _accel;
}

vector3f* AP_INS::raw_gyro()
{
	return _raw_gyro;
}

vector3f* AP_INS::raw_accel()
{
	return _raw_accel;
}


void     AP_INS::setHil(const float &_rr, const float &_pr, const float &_yr, const float &_xacc, const float &_yacc, const float &_zacc)
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
	_raw_gyro[0]   = _gyro[0]    *  (degreesf() * (1/0.015625f)); // LSB //_raw_gyro[MPU6000]  = _gyro[MPU6000]   *  (degreesf() * 65.5f);   // LSB
	_raw_accel[0]  = _accel[0]   *  (8192.0f/9.81f); 	          //_raw_accel[MPU6000] = _accel[MPU6000]  * (8192.0f/9.81f); 		// LSB
}

bool  	 AP_INS::have_ins()
{
	return _have_ins;
}

uint32_t AP_INS::last_update_msec()
{
	return _last_update_msec;
}

bool AP_INS::healthy()
{
	return ((millis() - _last_update_msec) < 1100) && _have_ins; // delay in sending ins message to the gcs might cause raising this error
}
