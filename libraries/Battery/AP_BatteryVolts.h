#pragma once
#include "Arduino.h"

class AP_BatteryVolts{
	public:
	AP_BatteryVolts():
	sum_adc(0),
	count(0),
	volts(0)
	{
		
	}
	
	void initialise();
	void read();
	float get_adc();
	
	private:
	uint16_t sum_adc;
	byte count;
	float volts;
};