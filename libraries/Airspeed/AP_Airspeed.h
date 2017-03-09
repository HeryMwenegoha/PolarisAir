#ifndef AP_Airspeed_h
#define AP_Airspeed_h
#include "MS4525DO.h"

#define MS4525DO	1

class AP_Airspeed
{
	public:
	AP_Airspeed(uint8_t *_enable):
	_hil_mode(false),
	_temperature(20.0f),
	_airspeed(0.0f),
	_raw_airspeed(0.0f),
	_raw_temperature(0.0f),
	enable(_enable),
	usable(false),
	_AP_MS4525DO(&AP_ms4525do)
	{};
	
	void  initialise();
	void  read();
	bool  enabled();

	
	float get_airspeed();
	float get_temperature();
	void setHil(float airspeed);
	static constexpr float PSI_Pa = 6894.757f;
	
	private:
	bool  calibrate();
	float Poffset; 
	
	
	protected:
	AP_MS4525DO	*_AP_MS4525DO;
	float _raw_airspeed;
	float _raw_temperature;
	float _airspeed;
	float _temperature;
	bool _hil_mode;
	bool   usable;
	uint8_t  *enable;
};


#endif