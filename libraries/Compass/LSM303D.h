#pragma once
#include "AP_Compass_Backend.h"
#include "Wire.h"
#include "Vectors.h"

class AP_LSM303D : public AP_Compass_Backend
{
	public:
	AP_LSM303D():
	_backend(&AP_compass_backend)
	{
		_last_time_us = 0;
		_dt_mag = 0;
		_update_us = 0;
		_have_sens = false;
	}
	
	void initialise();
    void accumulate();
	void update();
	
	private:
	float    _dt_mag;
	uint64_t _update_us;
	uint64_t _last_time_us;
	
	bool _have_sens;
	
	AP_Compass_Backend *_backend;
};