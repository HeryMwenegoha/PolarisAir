#include "AP_Compass.h"
AP_Compass AP_compass;

void AP_Compass::update()
{
	// Read Compass at the prescribed compass speed and update mag values at 10Hz.
	// 10Hz update cycles
	if(_hil_mode == true)
		return;
			
	//_raw_mag.printV() ; 					// LSB	
	
	
	//float mag_scale_xy = 1/855.0f;			// POLOLU XY GASS/LSB 1.9GAUSS
	//float mag_scale_z  = 1/760.0f;			// POLOLU Z GAUSS/LSB 1.9GAUSS
	
	float mag_scale_xy = 1/1100.0f;			// POLOLU XY GASS/LSB 1.3GAUSS
	float mag_scale_z  = 1/980.0f;			// POLOLU Z GAUSS/LSB 1.3GAUSS
	
	//float mag_scale    = 0.00091743f;		// MPU - GAUSS/LSB	
	
	// For LSM303D
	_mag_field.x = _raw_mag.x * mag_scale_xy;
	_mag_field.y = _raw_mag.y * mag_scale_xy;
	_mag_field.z = _raw_mag.z * mag_scale_z;
	
	_last_update_msec = millis();
	
	
	if(_raw_mag.is_zero()){
		_have_compass = false;
		//Serial.println("AP_Compass:: No Compass Detected");
	}else{
		_have_compass = true;
	}
				
	
	/*
	Serial.print(_mag_field.x,4);
	Serial.print("	");
	Serial.print(_mag_field.y, 4);
	Serial.print("	");
	Serial.println(_mag_field.z, 4);
	*/
	
	/*
	Serial.print(_raw_mag.x);
	Serial.print("	");
	Serial.print(_raw_mag.y);
	Serial.print("	");
	Serial.println(_raw_mag.z);
	*/		
					
}

vector3f AP_Compass::get_field()
{
	return _mag_field;
}

vector3f AP_Compass::raw_field()
{
	return _raw_mag;
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
	
	uint32_t _current_time = millis();
	if(_current_time - _update_msec >= 50)
	{
		_update_msec  = _current_time;
		_mag_field 	  = B_body; 			// GAUSS value
		_raw_mag      = _mag_field * 1090;	// LSB   value refer to Default Gain of HMC5883
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

