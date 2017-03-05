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
		_raw_mag.zero();
		_filtered_adc.zero();
		_low_pass_cutoff = 5; // 5Hz speed
	}
		
	void 	 update();
	vector3f get_field();
	vector3f raw_field();
	float 	 get_declination();
	float 	 calculate_heading(const float &_roll, const float &_pitch);	
	void  	 setHil(const float &hilroll, const float &hilpitch, const float &hilyaw);
	bool  	 have_compass();
	uint32_t last_update_msec();
	
	protected:
	vector3f _mag_field;
	bool _have_compass;
	bool _hil_mode;
	uint32_t _last_update_msec;
	uint32_t _update_msec;
	
	private:
	bool _new_data;
	vector3f _raw_mag;
	vector3f _filtered_adc;
	float _low_pass_cutoff;
};

extern AP_Compass AP_compass;