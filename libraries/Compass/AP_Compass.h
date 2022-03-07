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
		// ADC initialise
		_mag_adc.zero();
		_raw_mag_adc.zero();
		_filtered_mag_adc.zero();
		
		// LPF cut-off Hz
		_low_pass_cutoff   = 5; // 5Hz speed
		
		// Registered device
		registered_devices = 0;
		
		// device selection
		instance           = FXOS8700;
		
		// Standard
		value.min 	  = vector3f( -200,  -200,  -200); // 
		value.max 	  = vector3f(  200,   200,  -200); // check if LSB
		
		// LSM303D
		value.min 	  = vector3f( -255,  -298,  -724); // check if LSB
		value.max 	  = vector3f(  441,   377,  -91);
		
		// FXO82700
		value.max     = vector3f( 967,  -333,  -292);  // LSB confirmed
		value.min     = vector3f(-520, -1302, -1223);  // LSB confirmed
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
	
	
	uint8_t  registered_devices;
	
	protected:
	vector3f _mag_field; 	 // uT
	vector3f _raw_mag_field; // uT
	
	bool     _have_compass;
	bool     _hil_mode;
	uint32_t _last_update_msec;
	uint32_t _update_msec;
	void      determine_max(vector3f &value1, vector3f &value2);
	void      determine_min(vector3f &value1, vector3f &value2);
	
	private:
	int8_t    instance;
	bool 	 _new_data;
	
	vector3f _mag_adc;      	// LSB filtered and directly used
	vector3f _raw_mag_adc;  	// LSB to return raw_field
	vector3f _filtered_mag_adc; // LSB filtered and only passes value to _mag_adc
	
	float    _low_pass_cutoff;
	
	struct _value{
		vector3f max;
		vector3f min;
	};
	_value value;
};

extern AP_Compass AP_compass;