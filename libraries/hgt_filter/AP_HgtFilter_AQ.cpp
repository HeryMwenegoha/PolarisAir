#include "AP_HgtFilter_AQ.h"
void AP_HgtFilter_AQ::update(){
		// Get altitude estimate
	uint64_t tnow = micros();
	float DT      = (tnow - _update_last_usec) * 1e-6f;
	
	if(DT >= 0.020){
		_update_last_usec  = tnow;
	  
		if(DT > 1.0f)
		{
			EstAlt       = _ahrs.baro().get_altitude();
			EstVelocity  = 0.0f;
			AltErrorI    = 0.0f;
			DT           = 0.02f; // 50Hz
		}   	
		
		if(_ahrs.healthy() == false){
			return;
		}
				
		float Kp1      = 0.55f ;   // PI observer velocity gain 0.55
		float Kp2      = 1.0f;     // PI observer position gain 1.00
		float Ki       = 0.001f;   // PI observer integral gain (bias cancellation) 0.001
		float AltError = 0.0f;
		float InstAcc  = 0.0f;
		float Delta    = 0.0f;
		
		/*
		float Kp1      = _params->ParameterStorage.list.arspdEnabled; 		// 2.00f ;   // PI observer velocity gain 0.55
		float Kp2      = _params->ParameterStorage.list.TECS_stallPrevent; // 1.0f;     // PI observer position gain 1.00
		float Ki       = _params->ParameterStorage.list.PowerModule_Gain; // 0.001f;   // PI observer integral gain (bias cancellation) 0.001
		*/
		
		if (!inited) {
			inited       = 1;
			EstAlt       = _ahrs.baro().get_altitude();
			EstVelocity  = 0.0f;
			AltErrorI    = 0.0f;
		}
		
		// LPF accelerometer data:

		// Estimation Error
		AltError     = _ahrs.baro().get_altitude() - EstAlt; 
		AltErrorI   += AltError;

		// Gravity vector correction and projection to the local Z
		// The negavtive just before the acceleration is to make the upward direction positive
		InstAcc      = -((_ahrs.dcm() * vector3f(_ahrs.acc.x, _ahrs.acc.y, _ahrs.acc.z)).z + 9.81) + (Ki) * AltErrorI;

		// Integrators
		Delta        = InstAcc * DT + (Kp1 * DT) * AltError;
		EstAlt      += ((EstVelocity + Delta * 0.5f) * DT + (Kp2 * DT) * AltError);
		EstVelocity += Delta;
		
		//_height      = 0.8 * _height + EstAlt * 0.2;
		//_climbrate   = 0.8 * _climbrate + EstVelocity * 0.2;
		
		#if PRINT
		Serial.print(DT,4);
		Serial.print("\t");
		Serial.print(millis());
		Serial.print("\t");
		Serial.print(EstAlt);
		Serial.print("\t");
		Serial.println(EstVelocity * 100);
		#endif
	}
}


float AP_HgtFilter_AQ::height(){
	return EstAlt;
}


float AP_HgtFilter_AQ::altitude(){
	return EstAlt;
}

float AP_HgtFilter_AQ::climbrate(){
	return EstVelocity;
}
