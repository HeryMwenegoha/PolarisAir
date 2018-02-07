#pragma once
//#include "NMEA.h"
#include "UBX.h"
#include "Vectors.h"
class AP_GPS
{
	public:
	AP_GPS():
		_have_gps(false),
		_hil_mode(false),
		_INSTANCES(0)
	{};
	void initialise(HardwareSerial *Port);	
	void initialise(HardwareSerial *Port, uint32_t baud);
	void read(void);
	void setHil(const float &lat, const float &lon, const float &alt, const vector3f velocity);
	void setHilHeading(const int16_t &heading);
	static const byte GPS_FIX_TYPE_NF =	0x00;
	static const byte GPS_FIX_TYPE_DR =	0x01;
	static const byte GPS_FIX_TYPE_2D =	0x02;
	static const byte GPS_FIX_TYPE_3D =	0x03;
	static const byte GPS_FIX_TYPE_3DR=	0x04;
	static const byte GPS_FIX_TYPE_TO =	0x05;
	static const byte GPS_MIN_SATS    = 6;
	
	static const byte GPS_FIX_OK 	  = 0x01;
	
	/*
	struct UTC{
		uint8_t hour;
		uint8_t minutes;
		uint8_t seconds;
	}_UTC;
	UTC time();
	*/
		
	bool 	 have_gps()const;
	uint8_t  status()const;
	uint8_t  state()const;
	uint8_t  num_sats()const;
	uint16_t horizontal_dilution();   
	uint16_t vertical_dilution();    
	uint32_t last_time_fix_ms()const;
	float 	 altitude()const; // msl
	float  	 heading()const;
	float 	 groundspeed()const;
	
	uint8_t hours();
	uint8_t minutes();
	uint8_t seconds();
	

	struct 	 _loc
	{
		float lat;
		float lon;
	};
	struct _loc location()const;
	vector3f velocity()const;
	uint32_t BaudRate();
	bool     healthy();
	
	private:
	bool _hil_mode;
	GPSUBX	_gpsubx;
	#if HIDE
	GPSNMEA	_gpsnmea;
	#endif
	
	uint8_t _hours;      // hour in UTC : EAST = UTC + 3
	uint8_t _minutes;   // minutes
	uint8_t _seconds;   // seconds	
	
	bool _have_gps;
	uint8_t  _status;
	uint8_t  _state;
	uint8_t  _num_sats;
	uint16_t _horizontal_dilution;   
	uint16_t _vertical_dilution;    
	uint32_t _last_time_fix_ms;
	float 	 _height;
	float  	 _heading;
	float 	 _groundspeed;
	struct _loc _location;
	vector3f _velocity;
	byte _INSTANCES;
};