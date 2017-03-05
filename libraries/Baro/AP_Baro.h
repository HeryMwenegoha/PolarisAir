#pragma once
#include "BMP180.h"
class AP_Baro
{
	public:
	AP_Baro():
		_have_sens(false),
		_healthy(false),
		_hil_mode(false),
		_altitude(0),
		temp(25),
		press(1000)
		
	{
		_hil_msec = 0;
		_last_read_usec = 0;		
	}
	void initialise();
	void read();  	 	// 50Hz
	void update(); 		// 10Hz
	
	float get_altitude(); 		// metres
	float get_pressure(); 		// mb
	float get_temperature();	// Celsius
	
	void  setHil(float);
	bool  have_sens();
	bool  healthy();
	
	protected:
	float _altitude, temp, press;
	float _last_temperature;
	float _last_pressure;
	
	private:
	bool _hil_mode;
	bool _have_sens;
	bool _healthy;
	uint32_t _hil_msec;
	SFE_BMP180	_bmp180;
	uint64_t _last_read_usec;
};