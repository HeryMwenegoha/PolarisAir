#pragma once
#include "AP_AHRS.h"
#include "AP_Parameters.h"

class AP_HgtFilter_AQ{
	public:
	AP_HgtFilter_AQ(AP_AHRS &ahrs, AP_Storage *params):
	_ahrs(ahrs),
	_params(params),
	_update_last_usec(0),
    inited(0),
    AltErrorI(0.0f),
    AccScale(0.0f),
    EstVelocity(0.0f),
    EstAlt(0.0f)
	{
		
	}
	
	// runs at 40Hz
	void  update();
	float height();
	float altitude();
	float climbrate();
	
	private:
	uint64_t _update_last_usec;
	AP_AHRS &_ahrs;
	AP_Storage *_params;
    uint8_t inited;
    float AltErrorI;
    float AccScale;
    float EstVelocity;
    float EstAlt;
    float _height;
   float _climbrate;
};
