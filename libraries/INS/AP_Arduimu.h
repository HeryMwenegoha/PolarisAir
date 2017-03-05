#pragma once
#include "AP_INS_Backend.h"
#include "AP_Compass_Backend.h"

class AP_Arduimu : public  AP_INS_Backend, public AP_Compass_Backend
{
	public:
	AP_Arduimu():
	_ins_backend(&AP_ins_backend),
	_compass_backend(&AP_compass_backend)	
	{	
		_update_msec      = 0;
		_last_update_msec = 0;
		confirmed_pax = false;
		index = 0;
	};
	
	// Uses Serial2 for communicating with the device
	void initialise();//HardwareSerial *_Port);	
	void accumulate();
	void update();
		
	private:	
	//HardwareSerial *Port;
	AP_INS_Backend *_ins_backend;
	AP_Compass_Backend *_compass_backend;
	void every_second();
	
	uint32_t _last_update_msec;
	uint32_t _update_msec;
	
	typedef struct _message
	{
	 uint64_t time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	 int16_t xacc; ///< X acceleration (raw)
	 int16_t yacc; ///< Y acceleration (raw)
	 int16_t zacc; ///< Z acceleration (raw)
	 int16_t xgyro; ///< Angular speed around X axis (raw)
	 int16_t ygyro; ///< Angular speed around Y axis (raw)
	 int16_t zgyro; ///< Angular speed around Z axis (raw)
	 int16_t xmag; ///< X Magnetic field (raw)
	 int16_t ymag; ///< Y Magnetic field (raw)
	 int16_t zmag; ///< Z Magnetic field (raw)
	} _message_t;
	
	union decode{
		uint8_t buffer[sizeof(_message_t)];
		_message_t	message;
	};
	
	bool confirmed_pax;
	uint16_t index;
	uint8_t buffer[34];

	bool serialhelper(uint8_t);
	
	/*
	gyro_scale  = ToRad(0.0152672f);	// RADIANS_PER_SECOND/LSB
	mag_scale   = 0.00091743f;			// GAUSS/LSB	
	*/
};