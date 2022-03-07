#include "AP_Compass.h"
/*
 * By Hery Mwenegoha
 * Copyright 2016
 * Compass Activity Limited to +/- 1.3 Gauss
 * Allowance for only one Compass
 * If an external I2C Compass is connected on the I2C Port, it might corrupt manipulation
 */
AP_Compass AP_compass;


// Read Compass at the prescribed compass speed and update mag values at 10Hz.
// 10Hz update cycles
void AP_Compass::update()
{
	if(_hil_mode == true)
		return;
	
	float mag_scale_xy = 0;			
	float mag_scale_z  = 0;		
	
	switch(instance)
	{
		case LSM303D:
		mag_scale_xy  = 1.0f/1100.0f; 			
		mag_scale_z   = 1.0f/980.0f;		
		break;
		
		case HMC5883:
		mag_scale_xy  = 1.0f/1090.0f; 			
		mag_scale_z   = 1.0f/1090.0f;	
		break;
		
		case AK8963:
		mag_scale_xy  = 0.15f; 			
		mag_scale_z   = 0.15f;	
		break;
		
		case FXOS8700:
		mag_scale_xy  = 0.1; // LSB to uT 			
		mag_scale_z   = 0.1; // LSB to uT (microTesla) earth 25 - 65uT ; UK - 48uT		
		break;
	}		
	
	/*
	_raw_mag.x = _raw_mag_adc.x; // LSB // * mag_scale_xy; // Gives a +/- 1.3 Gauss Limit
	_raw_mag.y = _raw_mag_adc.y; // LSB // * mag_scale_xy;
	_raw_mag.z = _raw_mag_adc.z; // LSB // * mag_scale_z;
		
	// Convert to microTeslas easier to deal with
	//_raw_mag    = _raw_mag * 1000;
	*/
	
	//determine_max(_raw_mag, value.max);
	//determine_min(_raw_mag, value.min);
	vector3f  _adc_offsets = value.max + value.min;
	_adc_offsets 		   = _adc_offsets * 0.5f;
	
	vector3f  _adc_circle_scaling = value.max - value.min; 			// ellipse to circle based on LSB values
	_adc_circle_scaling			  = _adc_circle_scaling * 0.5f;	
	float avg_scale   = (_adc_circle_scaling.x + _adc_circle_scaling.y + _adc_circle_scaling.z) / 3;
	
	float x_scale     = avg_scale / _adc_circle_scaling.x;
	float y_scale     = avg_scale / _adc_circle_scaling.y;
	float z_scale     = avg_scale / _adc_circle_scaling.z;
	
	_mag_field.x 	  = (_mag_adc.x - _adc_offsets.x) * mag_scale_xy  * x_scale ; // uT  [filtered and corrected]
	_mag_field.y 	  = (_mag_adc.y - _adc_offsets.y) * mag_scale_xy  * y_scale ; // uT
	_mag_field.z	  = (_mag_adc.z - _adc_offsets.z) * mag_scale_z   * z_scale ; // uT
	
	_raw_mag_field.x  = _raw_mag_adc.x * mag_scale_xy; // uT  							[not filtered or corrected]
	_raw_mag_field.y  = _raw_mag_adc.y * mag_scale_xy; // uT
	_raw_mag_field.z  = _raw_mag_adc.z * mag_scale_z;  // uT
	
	// _mag_field.print();
	// Some Debug Prints
	// _raw_mag.print(0);
	// _raw_mag_adc.print(0);
	// _mag_field.print(0);
	
	_last_update_msec = millis();
	
	if(_raw_mag_field.is_zero())
	{
		_have_compass = false;
	}
	else
	{
		_have_compass = true;
	}				
}

vector3f AP_Compass::get_field()
{
	return _mag_field;     //also to log 
}

vector3f AP_Compass::raw_field()
{
	return _raw_mag_field; //log or something
}

float AP_Compass::get_declination()
{
	if(_hil_mode == true)
		return ToRad(18.0f);
	else
		return ToRad(3.0f);
}

float AP_Compass::calculate_heading(const float &_roll, const float &_pitch)
{
	float Heading; 
	float Head_X;
	float Head_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch; 
	
	// If Hil mode return field as is
	// If not return processed field
	// i.e. scaled, off-setted etc.
	vector3f mag_vector = AP_Compass::get_field();
	  
	cos_roll  = cos(_roll);
	sin_roll  = sin(_roll);
	cos_pitch = cos(_pitch);
	sin_pitch = sin(_pitch);

	// calculate the heading rotated to r and p and rotate the corresponding earth vector by -yaw to match this.
	Head_X  = mag_vector.x * cos_pitch  +  mag_vector.y * sin_roll * sin_pitch  +  mag_vector.z *cos_roll * sin_pitch;
	Head_Y  = mag_vector.y * cos_roll   -  mag_vector.z * sin_roll;
	Heading = atan2(-Head_Y,Head_X);


	// Angle normalization (-180 deg, 180 deg)
	Heading = Heading + AP_Compass::get_declination();
	if (Heading > M_PI)         
	  Heading -= (2.0 * M_PI);
	else if (Heading < -M_PI)
	  Heading += (2.0 * M_PI);
		  
	// limit heading to +/- 180 degrees
	Heading  = constrain_float(Heading, -M_PI, M_PI);
	
	//Serial.println(Heading * 180/M_PI);
	
	return Heading;	
}


void  AP_Compass::setHil(const float &hilroll, const float &hilpitch, const float &hilyaw)
{
	// All angles are in radians
	// 10Hz update rate of the magnetic field vector
	// Here the magnetic field vector is the proper dummy field and not the ADC like the SerialUpdate even though there is no need to get the proper magnetic field
	// Here it is also corrected for Direction into NED
	_hil_mode 				= true;
	_have_compass 			= true; 
	float dummy_field 	    = 0.53f;       // Field at SEATTLE - GAUSS
	float dummy_inclination = 67.0f;       // Inclination at SEATTLE 
	matrix3f R;
	vector3f B_earth;
	vector3f B_body;
	B_earth.x = dummy_field * cos(ToRad(dummy_inclination)); // inclination at SEATTLE KSEA
	B_earth.y = 0.0f;
	B_earth.z = dummy_field * sin(ToRad(dummy_inclination));// inclination at SEATTLE KSEA	
	R.from_euler(hilroll, hilpitch,hilyaw);
	B_body     = R.mul_transpose(B_earth);
	instance   = HMC5883;
	uint32_t _current_time = millis();
	if(_current_time - _update_msec >= 50)
	{
		_update_msec  = _current_time;
		_mag_field 	  = B_body; 			// GAUSS value
		_raw_mag_field    = _mag_field * 1090;	// LSB   value refer to Default Gain of HMC5883 _raw_mag     
		_last_update_msec = millis();
	}
}

bool AP_Compass::have_compass()
{
	return _have_compass;
}

uint32_t AP_Compass::last_update_msec()
{
	return _last_update_msec;
}


void AP_Compass::determine_max(vector3f &value1, vector3f &value2){
	
	bool trigger = false;
	
	if(value1.x > value2.x){
		value2.x = value1.x;
		trigger = true;
	}
	
	if(value1.y > value2.y){
		value2.y = value1.y;
		trigger = true;
	}
	
	if(value1.z > value2.z){
		value2.z = value1.z;
		trigger = true;
	}
	
	if(trigger){
		Serial.print(F("Max:	"));
		value2.print(0);
	}
}

void AP_Compass::determine_min(vector3f &value1, vector3f &value2){
	bool trigger = false;
	
	if(value1.x < value2.x){
		value2.x = value1.x;
		trigger = true;
	}
	
	if(value1.y < value2.y){
		value2.y = value1.y;
		trigger = true;
	}
	
	if(value1.z < value2.z){
		value2.z = value1.z;
		trigger = true;
	}
	
	if(trigger){
		Serial.print(F("Min:	"));
		value2.print(0);
	}
}
