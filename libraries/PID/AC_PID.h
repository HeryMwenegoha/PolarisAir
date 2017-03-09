#pragma once
#include "AP_Parameters.h"
#include "PID_INFO.h"

class AC_PID
{
	public:
	AC_PID(AP_Storage *params, uint8_t type):
	_input(0),
	_derivative(0),
	_dt(10),
	_last_update_us(0),
	_params(params),
	FilterFrequency(20),
	_output(0)
	{
		switch(type)
		{
			case ROLLPID:
				gains.kp      = &(_params->ParameterStorage.list.roll_kp);	
				gains.ki 	  = &(_params->ParameterStorage.list.roll_ki);	
				gains.kd 	  = &(_params->ParameterStorage.list.roll_kd);	
				gains.imax    = (float *)&(_params->ParameterStorage.list.roll_imax);	
				break;
			
			case PITCHPID:
				gains.kp      = &(_params->ParameterStorage.list.pitch_kp);	
				gains.ki 	  = &(_params->ParameterStorage.list.pitch_ki);	
				gains.kd 	  = &(_params->ParameterStorage.list.pitch_kd);	
				gains.imax    = (float *)&(_params->ParameterStorage.list.pitch_imax);	
				break;
			
			case YAWPID:
				gains.kp      = &(_params->ParameterStorage.list.steer_kp);	
				gains.ki 	  = &(_params->ParameterStorage.list.steer_ki);	
				gains.kd 	  = &(_params->ParameterStorage.list.steer_kd);	
				gains.imax    = (float *)&(_params->ParameterStorage.list.steer_imax);	
				break;
		}		
	}
	
	float get_pid(float );
    void  reset();
	
	private:
	float _input;
	float _derivative;
	float FilterFrequency;
	float _output;
	float _dt;
	
	uint64_t _last_update_us;
	pid_info _pid_info;
	_gains   gains;
	
	protected:
	AP_Storage *_params;
};