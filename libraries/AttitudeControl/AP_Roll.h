#pragma  once
#include "AP_Parameters.h"
#include "Common.h"
#include "AHRS.h"
#include "PID_INFO.h"
#include "AP_AHRS.h"

class AP_Roll
{
	public: 
	AP_Roll(){

	};
	void initialise(AP_Storage *, AP_AHRS *);
	float servo_out(float, float);
	
	private:
	float _rate_out(float, float);   // deg/sec;			
	pid_info _pid_info;
	_gains gains;
	
	float    _output;
	uint32_t _last_Msec;
	
	protected:
	AP_AHRS *_ahrs;	
};