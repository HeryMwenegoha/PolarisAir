#include "AP_Baro.h"
void AP_Baro::initialise()
{
	// Need to implement a wire taking mechanism in the core library
	//_have_sens = _bmp180.begin();	
	_have_sens   = _bmp388.begin();	
	if(_have_sens == false){
		Serial.println(F("AP_Baro::	BMP	Not	Detected"));
	}
	else{
		Serial.print(F("AP_Baro::	BMP	Ready,	Baseline	Pressure:	"));
		//Serial.println(_bmp180.baseline_pressure());
		Serial.println(_bmp388.baseline_pressure());
	}
}


// As fast as possible
void AP_Baro::read()
{
	if(_hil_mode == true)
		return;
	
	_bmp388.read();
	
	/*
	uint64_t now = micros();
	if((now - _read_usec) >= 10000)
	{
		_read_usec = now;
		_bmp180.read();
	}
	*/
}


void AP_Baro::update()
{
	_healthy = _bmp388.pressure(press)  && _bmp388.temperature(temp);
	//_healthy = _bmp180.pressure(press)  && _bmp180.temperature(temp);

	if(_healthy)
	{
		_altitude 		= (44330.0*(1-pow(press/_bmp388.baseline_pressure(),1/5.255)));	
		_timeStamp_msec = _bmp388.timeStamp_msec();
	}
	
	/*
	 * BMP180 Code
	if(_healthy)
	{
		uint64_t now    = micros();		
		float _dt       =(now - _last_read_usec) * 1e-6f;
		_last_read_usec = now; 
		
		if(_dt > 1.0f){
			_dt = 0.1f;
		}
		
		float _raw_alt = 0;
		float FC       = 20; // 5Hz changes
		float RC 	   = 1/(2 * PI * FC);
		float _alpha   = _dt/(_dt + RC);				
		
		// BaroNorm
		_raw_alt 	   = (44330.0*(1-pow(press/_bmp180.baseline_pressure(),1/5.255)));

		// BaroForm
		float TempK    = temp + 273.15;
		float scaling  = press/_bmp180.baseline_pressure();
		float altitude = 153.8426f * TempK * (1.0f - expf(0.190259f * logf(scaling)));
				
		if(isnan(_raw_alt))
		{
			_raw_alt = 0;
			return;
		}
		
		_altitude  = _raw_alt;
		
		//_altitude  = 0.8f * _altitude + _raw_alt * 0.2f;// _altitude + (_raw_alt - _altitude) * _alpha;
		// Matlab Analysis File : Press_Temp_BaroRaw_BaroISA
		#if DEBUG
		Serial.print(press);
		Serial.print("	");
		Serial.print(temp);
		Serial.print("	");
		Serial.print(_raw_alt);
		Serial.print("	");
		Serial.println(altitude);
		#endif
	}
	*/	
}

bool  AP_Baro::have_sens()
{
	return _have_sens;
}

bool  AP_Baro::healthy()
{
	return _healthy;
}

float AP_Baro::get_altitude()
{
	return _altitude;
}

uint32_t AP_Baro::get_timeStamp()
{
	return _timeStamp_msec;
}

float AP_Baro::get_pressure()
{
	return press/100; // mbar - BMP388
}

float AP_Baro::get_temperature(){
	return temp;
}


void  AP_Baro::setHil(float alt){
	_hil_mode  = true;
	_healthy   = true;
	_have_sens = true;
	
	// Update altitude at 77ms
	uint32_t now   = millis();
	if((now - _hil_msec) >= 100){
		_hil_msec  = now;
		_altitude  = alt;
	}
}
