#pragma once
#include "AP_Compass_Backend.h"
#include "Wire.h"
#include "Vectors.h"

/* Gives +/- 4800 microTeslas equivalent to 0.48Gauss
 * Sensitivity is 0.6uT/LSB 
 * 8hz updates rate
 */
class AP_AK8963 : public AP_Compass_Backend
{
	public:
	AP_AK8963():
	_backend(&AP_compass_backend)
	{
		_last_time_us = 0;
		_dt_mag       = 0;
		_update_us    = 0;
		_have_sens    = false;
		number_of_readings = 6;
		
		ASA.zero();
		
		_offset.min  = vector3f(-278, -24,  -443);
		_offset.max  = vector3f(126,  364,   -20);
		_offset.update();
		
		//value.min = vector3f(-370, -128,  -452);
		//value.max = vector3f( 376, 381, 117);
		
		//value.min = vector3f( -279,  2,  -464);
		//value.max = vector3f( 135,  389, -65);
				
	}
	
	void initialise();
    void accumulate();
	void update();
	
	private:
	struct offset{
		vector3f min;
		vector3f max;
		vector3f main;
		void update(){
		main = (min+max)*0.5f;	
		}
	};
	offset	 _offset;
	float    _dt_mag;
	uint64_t _update_us;
	uint64_t _last_time_us;
	uint8_t   number_of_readings;
	vector3f  ASA;
	void determine_max(vector3f & , vector3f &);
	void determine_min(vector3f &, vector3f &);
	
	bool _have_sens;
	AP_Compass_Backend *_backend;
};