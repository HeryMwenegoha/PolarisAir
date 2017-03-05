#ifndef NMEA_h
#define NMEA_h
#include "Arduino.h"

#define GPRMC "GPRMC"
#define GPGGA "GPGGA"

#define OTHER_SENTENCE 0
#define GPRMC_SENTENCE 1
#define GPGGA_SENTENCE 2
#define COMBINE(SENTENCE_TYPE, SEQUENCE) (((unsigned)(SENTENCE_TYPE << 5)) | SEQUENCE)

class GPSNMEA
{
	public:
	GPSNMEA();
	void initialise(HardwareSerial *Port, const uint16_t BaudRate);
	void process_stream();
	
	private:
	HardwareSerial *Port;
    //String GPRMC;
	//String GPGGA;
	void process_packet();
	int packet_sequence;
	uint8_t sentence_type;
	char buffer[15];
	bool _sentence_has_fix;
	bool _valid_fix;
	byte parity;
	bool _ischecksum;
	
	int fromHex(const char a);
	uint32_t ParseDegrees(char *buf);
};
#endif