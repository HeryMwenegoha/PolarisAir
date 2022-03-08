#pragma once
#include "AP_INS.h"
#include "Vectors.h"
 

class AP_INS_Backend2
{
	public:
	AP_INS_Backend2():
	_imu(&AP_ins)
	{};
    void    update_gyro(uint8_t instance);
	void    update_accel(uint8_t instance);
	void    filter_raw_sample_gyro(uint8_t instance, const vector3f &, float);
	void    filter_raw_sample_accel(uint8_t instance, const vector3f &, float);
	
	float    SCALER;    // [GYRO - LSB2DPS, ACCEL - LSB2G]
	uint8_t  TYPE;		// [SENSOR TYPE]
	char     CHAR;      //  SENSOR CHAR 'A' : Accel 'G' : Gyro
	
	protected: 
	AP_INS 	  *_imu; // Give the backend access to front end
};