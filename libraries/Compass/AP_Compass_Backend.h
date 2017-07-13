#pragma once
#include "AP_Compass.h"
#include "Vectors.h"

class AP_Compass_Backend
{
	public:
	AP_Compass_Backend():
	_compass(&AP_compass)
	{}
	
	void register_device(int8_t instance);
	
	void update_magnetometer();
	void filter_raw_sample(const vector3f &_raw_, float dt);
	
	protected:
	AP_Compass *_compass;
};

extern AP_Compass_Backend AP_compass_backend;