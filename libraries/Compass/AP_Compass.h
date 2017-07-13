#pragma once
//#include "AP_Arduimu.h"
#include "Vectors.h"
#include "Common.h"


class AP_Compass_Backend;

class AP_Compass
{
	friend class AP_Compass_Backend;
	
	public:
	AP_Compass():
	_hil_mode(false),
	_have_compass(false),
	_new_data(false)
	{
		_raw_mag_adc.zero();
		_filtered_adc.zero();
		_low_pass_cutoff   = 5; // 5Hz speed
		instance           = -1;
		registered_devices = 0;
		
		// LSM303D
		value.min 	  = vector3f( -255,  -298,  -724);
		value.max 	  = vector3f(  441,   377,  -91);
		
		// Standard
		//value.min 	  = vector3f(  -200,  -200,  -200);
		//value.max 	  = vector3f(   200,   200,   -200);
	}
	
	//void     initialise();
	void 	 update();
	vector3f get_field();
	vector3f raw_field();
	float 	 get_declination();
	float 	 calculate_heading(const float &_roll, const float &_pitch);	
	void  	 setHil(const float &hilroll, const float &hilpitch, const float &hilyaw);
	bool  	 have_compass();
	uint32_t last_update_msec();
	
	int8_t   instance;
	uint8_t  registered_devices;
	
	protected:
	vector3f _mag_field;
	bool _have_compass;
	bool _hil_mode;
	uint32_t _last_update_msec;
	uint32_t _update_msec;
	void determine_max(vector3f &value1, vector3f &value2);
	void determine_min(vector3f &value1, vector3f &value2);
	
	private:
	bool 	 _new_data;
	vector3f _raw_mag_adc;
	vector3f _filtered_adc;
	vector3f _raw_mag;
	float    _low_pass_cutoff;
	
	struct _value{
		vector3f max;
		vector3f min;
	};
	
	_value value;
};

extern AP_Compass AP_compass;