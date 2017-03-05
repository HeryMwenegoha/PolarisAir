#pragma once
#include "AP_Parameters.h"
#include "PID_INFO.h"

class AC_P{
	public:
	AC_P(AP_Storage *params, uint8_t type):
	_params(params)
	{
		switch(type)
		{
			case ROLLPID:
				gains.tau	  = &(_params->ParameterStorage.list.roll_tau);
				break;
			
			case PITCHPID:
				gains.tau	  = &(_params->ParameterStorage.list.pitch_tau);
				break;
			
			case YAWPID:
				gains.tau	  = &(_params->ParameterStorage.list.steer_tau);
				break;
		}		
	}
	
	float get_p(float );
	
	private:
	pid_info _pid_info;
	_gains   gains;
	
	protected:
	AP_Storage *_params;
};