#pragma once
#include "Arduino.h"

class AP_MS4525DO
{
	public:
	AP_MS4525DO():
	Pmax(1.0f),
	Pmin (-1.0f),
	ratio(1.67f),
	pressure(0.0f),
	last_Msec  (0.0f),
	Do_Event (1)
	//_scheduler(&AP_scheduler)
	{};
	bool  init();
	void  measure(); //  performs the background measuring of differential pressure
	void  measure(byte del_msec); //  performs the background measuring of differential pressure
	
	bool  get_differential_pressure(float &);
	bool  get_temperature(float &);

	private:
	bool  calibrate(); // offset return 
	void  startmeasurement();
	bool  collectdata();	
	
	float temperature; // Celsius
	float pressure;    // Pa
	
	float Pmax;
	float Pmin;	
	float ratio;
	
	byte  counter;
	byte  Do_Event;
	uint32_t last_Msec;
	uint32_t last_sample_time_msec;	
	//AP_Scheduler *_scheduler;
	
	//typedef void (AP_MS4525DO::*MethodPtr)();
	//typedef void (AP_Scheduler::*fPtr)(void(*isr)());
};

extern AP_MS4525DO AP_ms4525do;