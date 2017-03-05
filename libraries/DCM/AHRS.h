#ifndef AHRS_h
#define AHRS_h

#include "AP_Parameters.h"
#include "Arduino.h"
#include "Vectors.h"
#include "Common.h"

class AP_AHRS_BASE
{
	public:
	AP_AHRS_BASE():
	KP_ROLLPITCH(0.2f),  
	KI_ROLLPITCH(0.0087),
	KP_YAW(0.2f),  		 
	KI_YAW(0.01f), 
	KP_ROLLPITCH_MIN(0.05),
	KP_YAW_MIN(0.05),
	GPS_SPEED_MIN(3.0),
	SPIN_RATE_LIMIT(20.0),
	WIND_ESTIMATE_TRUST(0.75f),
	WIND_MAX(0),
	roll(0.0f),
	pitch(0.0f),
	yaw(0.0f),
	rollDeg(0.0f),
	pitchDeg(0.0f),
	yawDeg(0.0f)
	{
		_flags.correct_centrifugal = true;
		_flags.wind_estimation = true;
		_flags.fly_forward = true;
		_flags.have_initial_yaw = false;
	}
	static constexpr float M_2PI = 2 * PI; 
	struct ahrs_flags
	{
		uint8_t fly_forward 		: 1;
		uint8_t correct_centrifugal : 1;
		uint8_t wind_estimation 	: 1;
		uint8_t have_initial_yaw	: 1;
	}_flags;
	
	void set_flyforward(bool b)
	{
		_flags.fly_forward = b;
	}
	
	void set_windestimation(bool b)
	{
		_flags.wind_estimation = b;
	}
	
	
	bool is_equal(float a, float b)
	{
		if(a == b) 
		return true;
		else
		return false;
	}
	
	float wrap_360(const float deg)
	{
		float res = fmod(deg, 360.0f);
		if(res < 0){
			res += 360.0f;
		}		
		return res;
	}
	
	float wrap_180(const float deg)
	{
		float res = wrap_360(deg);
		if(res > 180.0f){
			res -= 360.0f;
		}
		return res;
	}
	
	float wrap_2PI(const float rad)
	{
		float res = fmod(rad, M_2PI);
		if(res < 0){
			res +=  M_2PI;
		}		
		return res;
	}
	
	float wrap_PI(const float rad)
	{
		float res = wrap_2PI(rad);		
		if(res > M_PI){
			res -= M_2PI;
		}	
		return res;
	}
	
	void set_flightmode(const byte mode)
	{
		_flight_mode = mode;
	}
	
	byte get_flightmode()
	{
		return _flight_mode;
	}
	
	
	float roll,     pitch,       yaw;	  // radians Note yaw is +/- pi i.e. +/- 180 degrees 
	float rollrate, pitchrate,   yawrate; // radians/second
	
	protected:
	float KP_ROLLPITCH;
	float KI_ROLLPITCH;
	float KP_YAW;
	float KI_YAW;
	float KP_ROLLPITCH_MIN;
	float KP_YAW_MIN;
	float GPS_SPEED_MIN;
	float SPIN_RATE_LIMIT;
	float WIND_ESTIMATE_TRUST;
	float WIND_MAX;
	float rollDeg,  pitchDeg,    yawDeg;  // Degrees - Note yawDeg is 0 - 360 degrees	
	void  update_degress();
	byte  _flight_mode;
};

struct hil_ahrs
{
	float roll;
	float pitch;
	float yaw;
	float rollrate;
	float pitchrate;
	float yawrate;
	float xacc;
	float yacc;
	float zacc;
};

struct hil_gps
{
	Vector2f location;
	vector3f velocity;
	float altitude;
	uint64_t time_usec;
};

#endif