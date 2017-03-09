#include "AP_Airspeed.h"
// setup function
void  AP_Airspeed::initialise()
{
	_hil_mode = false;
	 bool det = _AP_MS4525DO->init(); // Interrupt is fired here
	 usable   = calibrate(); 		  // as well as getting p_offset
	 
	 if(usable == false)
		Serial.println("AP_Airspeed:: Airspeed Detected");
	  else
		Serial.println("AP_Airspeed:: No Airspeed Detected");

}

bool  AP_Airspeed::enabled()
{
	return usable && (*enable == 1);
}

bool AP_Airspeed::calibrate()
{
	int P_count = 0;
	int max_values  = 11;
	for(int i = 0; i < max_values; i++)
	{
		_AP_MS4525DO->measure(10);
		float diff_pressure;
		if(_AP_MS4525DO->get_differential_pressure(diff_pressure))
		{
			if(i == 0){}			 // through first reading
			else
			{
				P_count++;
				Poffset += diff_pressure; // PSI
			}
		}
		delay(10); // 20ms between reads - worst case error -> makse sure loop is running at 20ms
	}	
	Poffset = Poffset/P_count;
	if(P_count == (max_values-1))
		return true;
	else
	{
		return false;
	}
}

// void loop function
void  AP_Airspeed::read()
{
	#if MS4525DO
	float diff_pressure;
	
	_AP_MS4525DO->measure();
	
	if(_hil_mode)	// dont update in hil
		return;
	
	if(_AP_MS4525DO->get_differential_pressure(diff_pressure))	// PSI
	{
		diff_pressure   = Poffset - diff_pressure; 
		diff_pressure   = max(diff_pressure, 0);
		_raw_airspeed 	= sqrt(diff_pressure * 1.67 * PSI_Pa);
		_airspeed  		= 0.7f * _airspeed + 0.3f * _raw_airspeed;	
	}
	
	if(_AP_MS4525DO->get_temperature(_raw_temperature))  		// CELSIUS
	{
		_temperature 	= 0.7f * _raw_temperature + _temperature * 0.3f;
	}
	#endif
}


float AP_Airspeed::get_airspeed()
{
	return _airspeed;
}


float AP_Airspeed::get_temperature()
{
	return _temperature;
}

void AP_Airspeed::setHil(float airspeed)
{
	_hil_mode  = true; 
	usable	   = true;
	_airspeed  = airspeed;
}
