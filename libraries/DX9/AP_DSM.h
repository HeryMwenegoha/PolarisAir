#pragma once
#include "Arduino.h"

class AP_DSM
{
	private:
	HardwareSerial *Port;
	uint64_t 		_last_dsm_usec;
    uint64_t        _update_usec;
	uint8_t 		_dsm_shift;
	uint8_t 		samples;
	uint32_t 		cs10;
	uint32_t 		cs11;
	//uint8_t			_dsm_buffer[16];
	uint8_t 		decoded_channels;
	public:
	AP_DSM():
	_dsm_shift(0),
	samples(0),
	cs10(0),
	cs11(0),
	decoded_channels(0)
	{
		
	}
	
	uint16_t values[9];	
	float 	 DT;	
	uint16_t length;
	void 	 initialise(HardwareSerial *_Port);
	void 	 guess_format(bool reset_variables, const uint8_t dsm_buffer[16]);
	boolean  decode_channel(uint16_t _raw, uint16_t shift,  uint16_t *channel, uint16_t *value);
	boolean  decode_stream();
	void read_stream();
	
	uint16_t chan1(){
		return values[0];
	}
	uint16_t chan2(){
		return values[1];
	}
	uint16_t chan3(){
		return values[2];
	}
	uint16_t chan4(){
		return values[3];
	}
	uint16_t chan5(){
		return values[4];
	}
	uint16_t chan6(){
		return values[5];
	}
	uint16_t chan7(){
		return values[6];
	}
	uint16_t chan8(){
		return values[7];
	}
	uint16_t chan9(){
		return values[8];
	}
};