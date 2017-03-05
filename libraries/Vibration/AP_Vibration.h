#pragma once
#include "Vectors.h"

class AP_Vibration{
	public:
	AP_Vibration():
	VIBE_FLOOR_HPF_HZ(5),
	VIBE_DIFF_HPF_HZ(2)
	{
		
	}
	
	vector3f update(vector3f acc);
	
	private:
	vector3f  accel_last;
	vector3f  accel_floor;
	vector3f  accel_diff_last_sq;
	vector3f  vibe_sq;
	
    float VIBE_FLOOR_HPF_HZ;
    float VIBE_DIFF_HPF_HZ;
	uint64_t  _last_usec;
};