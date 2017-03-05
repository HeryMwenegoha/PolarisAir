#pragma once
#include "AP_INS.h"
#include "Vectors.h"
//#include "AP_Scheduler.h"
 

class AP_INS_Backend
{
	public:
	
	AP_INS_Backend();
	//virtual void accumulate() {}
    void    update_gyro(uint8_t instance);
	void    update_accel(uint8_t instance);
	void    filter_raw_sample_gyro(uint8_t instance, const vector3f &, float);
	void    filter_raw_sample_accel(uint8_t instance, const vector3f &, float);
	
	protected: 
	AP_INS 		 *_imu; // Give the backend access to front end
};

extern AP_INS_Backend AP_ins_backend;