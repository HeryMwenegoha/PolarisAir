#include "AP_Baro.h"
void AP_Baro::initialise()
{
	// Need to implement a wire taking mechanism in the core library
	_have_sens = _bmp180.begin();	
	if(_have_sens == false){
		Serial.println(F("AP_Baro::	BMP	Not	Detected"));
	}
	else{
		Serial.print(F("AP_Baro::	BMP	Ready,	Baseline	Pressure:	"));
		Serial.println(_bmp180.baseline_pressure());
	}
}


// As fast as possible
void AP_Baro::read()
{
	if(_hil_mode == true)
		return;

	_bmp180.read();
	
	// as fast as i can
	// if(_bmp180.read()){
	//	update();
	// }
}

void AP_Baro::update()
{
	_healthy = _bmp180.pressure(press)  && _bmp180.temperature(temp);

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
		
		_raw_alt 	   = (44330.0*(1-pow(press/_bmp180.baseline_pressure(),1/5.255)));
				
		if(isnan(_raw_alt))
		{
			_raw_alt = 0;
			return;
		}
		
		_altitude  = _raw_alt;
		
		//_altitude  = 0.8f * _altitude + _raw_alt * 0.2f;// _altitude + (_raw_alt - _altitude) * _alpha;
		
			
		#if PRINT
		Serial.print(press);
		Serial.print("	");
		Serial.print(temp);
		Serial.print("	");
		Serial.print(_raw_alt);
		Serial.print("	");
		Serial.println(_altitude);
		#endif
	}	
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

float AP_Baro::get_pressure(){
	return press;
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
