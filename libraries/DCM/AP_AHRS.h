#pragma once
#include "AHRS.h"
#include "Vectors.h"
#include "AP_Airspeed.h"
#include "AP_GPS.h"
#include "AP_Compass.h"
#include "AP_Baro.h"
#include "AP_INS.h"
#include "AP_Vibration.h"

#define NED 1
#define FLT_EPSILON	1.19209290e-07F
#define SPIN_RATE_LIMIT 20

class AP_AHRS : public AP_AHRS_BASE
{	
	private:
	// vectors
	vector3f gyro_vector;
	vector3f accel_vector;
	vector3f mag_vector;
	vector3f error_RollPitch;

	// dcm correction variables
	vector3f omega;
	vector3f omega_I_sum;
	vector3f omega_P;
	vector3f omega_I;
	vector3f omega_yaw_P;
	float G_dt;								// Loop Speed
	float omega_I_sum_time;
	float _ra_deltat;      					// amount of time we have integrated the _ra_sum matrix
	vector3f _ra_sum; 						// sum of dcm * accel_vector
	vector3f _ra_delayed(vector3f new_ra);
	vector3f _ra_delayed_buffer;
	uint32_t _ra_sum_start;
	
	// Compass correction
	uint32_t _compass_last_update_msec;
	uint32_t _gps_last_update_msec;
	
	// dead reckoning variables
	float _last_lat;
	float _last_lon;
	float _position_offset_north;
	float _position_offset_east;
	bool  _have_position;
	bool _have_gps_lock;
	vector3f _last_velocity;
	float _last_airspeed;
	
	// System Times
	uint32_t last_failure_Msec;
	uint32_t last_startup_Msec;
	uint32_t  update_Msec;
	uint32_t _last_consistent_heading;
	uint32_t _last_hil_millis;
	
	// Wind Estimates	
	uint32_t _last_wind_time;
	vector3f wind;
	vector3f fusDir_old;
	vector3f gSpeed_old;
	
	// Main direct cosine matrix
	matrix3f dcmMatrix;
	
	// Main Methods
	void  update_matrix(void);
	bool  renormalise(vector3f const &a, vector3f &result);
	void  normalise(void);
	float yaw_error_compass();
	bool  use_compass();
	void  drift_correction_yaw(void);
	void  drift_correction(void);
	void  check_matrix(void);
	void  euler_angles(void);
	void  wind_estimate(void);
	
	// Helper Methods
	void  reset_dcm(const boolean recover_eulers);		
	bool  use_fast_gains(void)const;
	float _P_gain(float _spin_rate);
	float _yaw_gain(void) const;
	bool   _hil_mode;
	
	// Pointer and References to Classes.
	AP_Airspeed		*_airspeed;
	const AP_GPS	&_gps;
	AP_INS			*_ins;
	AP_Compass 		*_compass;
	AP_Baro			&_baro;
	AP_Vibration	AP_vibration;
	public:
	AP_AHRS(AP_Airspeed *_AP_Airspeed, AP_GPS &_AP_GPS, AP_INS *_AP_INS, AP_Compass *_AP_Compass, AP_Baro &_baro_):
	AP_AHRS_BASE(),
	_airspeed(_AP_Airspeed),
	_gps(_AP_GPS),
	_ins(_AP_INS),
	_compass(_AP_Compass),
	_baro(_baro_),
	_have_position(false),
	_have_gps_lock(false),
	_hil_mode(false)
	{
	    dcmMatrix.identity();
	};

	
	void  	 update();
	bool  	 get_position(Location &Loc)const;
	Vector2f groundspeed_vector(); // zero means no valid measuremnt
	float 	 airspeed_estimate();  // zero means no valid measurement
	float    altitude_estimate();  // zero means no valid measurement
	void  	 setHil();
	bool  	 healthy(void)
			{
				return((last_failure_Msec == 0) || (millis()-last_failure_Msec > 5000)) && _ins->healthy();
			}
	vector3f acc;
	bool     isflying();
	vector3f vibe;
	matrix3f dcm()
	{
		return dcmMatrix;
	};
	
	const AP_GPS &gps(){
		return _gps;
	}
	
	AP_Baro &baro(){
		return _baro;
	}
	
	AP_Airspeed &airspeed(){
		return *_airspeed;
	}
	vector3f &_wind_estimate(){
		return wind;
	}
	
};