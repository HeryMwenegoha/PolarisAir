#pragma once
#include "AP_AHRS.h"
#include "AP_Parameters.h"

class AP_HgtFilter{
	public:
	AP_HgtFilter(AP_AHRS &ahrs, AP_Storage &params):
	_ahrs(ahrs),
	_params(params),
	_update_last_usec(0),
	_hgtCompFiltOmega(3.0),
	_integ3_state(0),
	_integ1_state(0),
	_climb_rate(0),
	gravity(9.80665)
	{
		
	}
	
	void  update();
	float height();
	float altitude();
	float climbrate();
	
	private:
	uint64_t _update_last_usec;
	AP_AHRS &_ahrs;
	AP_Storage &_params;
	float _hgtCompFiltOmega;
	float _integ3_state;
	float _integ1_state;
	float _climb_rate;
	float gravity;
};
