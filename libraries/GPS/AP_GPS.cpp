#include "AP_GPS.h"

void AP_GPS::initialise(HardwareSerial *Port)
{
	if(_gpsubx.initialise(Port))
		_INSTANCES++;
	
	if(_INSTANCES != 0)
		_have_gps = true;
	else
		_have_gps = false;
		
	if(_have_gps == false)
		Serial.println("AP_GPS::No	GPS	Device	Detected");
	else
	{
		Serial.print("AP_GPS::	GPS	on	Port	1,	Baud:	");
		Serial.println(AP_GPS::BaudRate());
	}
	
}

void AP_GPS::read(void)
{
	if(_gpsubx.process_stream() == true) // if we have new updates
	{
		if(_hil_mode == true)			// dont update in hil mode
			return;
			
		
		_status   = _gpsubx.UBX.fix_type;
		_state	  = _gpsubx.UBX.gpsFixOk;
		_num_sats = _gpsubx.UBX.numSV;
		_horizontal_dilution = _gpsubx.DOP.hDOP;   
		_vertical_dilution  = _gpsubx.DOP.vDOP;    
		_last_time_fix_ms = _gpsubx.UBX.fix_time_ms;
		
		
		_height = _gpsubx.UBX.hMSL;
		_heading = _gpsubx.UBX.heading;
		_groundspeed = _gpsubx.UBX.gSpeed;
		_location.lat =  _gpsubx.UBX.lat;
		_location.lon = _gpsubx.UBX.lon;
		_velocity.x = _gpsubx.UBX.velN;
		_velocity.y = _gpsubx.UBX.velE;
		_velocity.z = _gpsubx.UBX.velD;
		
		_hours 	 = _gpsubx.UBX.hour;
		_minutes = _gpsubx.UBX.minutes;
		_seconds = _gpsubx.UBX.seconds;
		
		
		
		/*
		Serial.print("GPS ");
		Serial.print(_location.lat, 7);
		Serial.print("	");
		Serial.print(_location.lon, 7);	
		Serial.print("	");
		Serial.print(_velocity.x, 2);	
		Serial.print("	");
		Serial.print(_velocity.y, 2);	
		Serial.print("	");
		Serial.println(_num_sats);
		*/
	}
}

void AP_GPS::setHil(const float &lat, const float &lon, const float &alt, const vector3f velocity)
{
	_hil_mode = true;
	
	uint32_t time = millis();
	if(time - _last_time_fix_ms >= 200)
	{
		// commit only after every 200 ms
		_last_time_fix_ms = time;
		_have_gps = true;
		_status   = AP_GPS::GPS_FIX_TYPE_3D;
		_state    = AP_GPS::GPS_FIX_OK;
		_num_sats = 7;
		_horizontal_dilution = 0.02f;   
		_vertical_dilution  = 0.03f;    		
		_height       = alt;
		_groundspeed  = velocity.length();
		_location.lat =  lat;
		_location.lon = lon;
		_velocity.x   = velocity.x;
		_velocity.y   = velocity.y;
		_velocity.z   = velocity.z;
		
		/*
		_UTC.hour     = _gpsubx.UTC.hour;
		_UTC.minutes  = _gpsubx.UTC.minutes;
		_UTC.seconds  = _gpsubx.UTC.seconds;
		*/
		
		/*
		Serial.print("V ");
		Serial.println(_groundspeed);
		*/
	}
}

void AP_GPS::setHilHeading(const int16_t &heading)
{
	_heading      = heading;
}

bool AP_GPS::have_gps()const
{
	return _have_gps && (_status > AP_GPS::GPS_FIX_TYPE_NF);
}

uint8_t AP_GPS::status()const
{
	return _status;
}

uint8_t AP_GPS::state()const
{
	return _state;
}

uint8_t AP_GPS::num_sats()const
{
	return _num_sats;
}

uint16_t AP_GPS::horizontal_dilution()
{
	return _horizontal_dilution;
}
  
uint16_t AP_GPS::vertical_dilution()
{
	return _vertical_dilution;
} 

uint32_t AP_GPS::last_time_fix_ms()const
{
	return _last_time_fix_ms;
}

// above mslv
float AP_GPS::altitude()const
{
	return _height;
}

// should be 0-360 degrees
float AP_GPS::heading()const
{
	return _heading;
}

float 	 AP_GPS::groundspeed()const
{
	return _groundspeed;
}

AP_GPS::_loc AP_GPS::location()const
{
	return _location;
}

vector3f 	AP_GPS::velocity()const
{
	return _velocity;
}

uint32_t	AP_GPS::BaudRate()
{
	return _gpsubx.get_BaudRate();
}

bool	    AP_GPS::healthy()
{
	return (millis() - _last_time_fix_ms) < 5000;
}

uint8_t 	AP_GPS::hours(){
	return _hours;
}

uint8_t 	AP_GPS::minutes(){
	return _minutes;
}

uint8_t 	AP_GPS::seconds(){
	return _seconds;
}