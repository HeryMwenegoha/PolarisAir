#include "NMEA.h"

GPSNMEA::GPSNMEA(){
	//GPRMC = "GPRMC";
	//GPGGA = "GPGGA";
}

void GPSNMEA::initialise(HardwareSerial *_Port, const uint16_t BaudRate)
{
    Port = _Port;
	Port->begin(BaudRate);
	packet_sequence = 0;
	buffer[0] 	    = 0;
	sentence_type   = 0;
}

void process_parity(byte *buffer)
{
	
}

void GPSNMEA::process_stream()
{
	int index = 0;
	int no_check  = 0;
	while(Port->available())
	{
		char c = static_cast<char>(Port->read());
		
		switch(c)
		{
			case ',':
			parity ^= (uint8_t)c;
			
			case '\r':
			
			case '\n':
									
			case '*':
			if(index < sizeof(buffer))
			{
				buffer[index] = 0;
				process_packet();
				//Serial.println(buffer);
			}
			_ischecksum = c == '*';   // checks whether we have received a checksum sign-> stop parity check if true
			index  = 0;
			packet_sequence++; 			// advance packet position
			break;
			
			case '$':
			// start character
			index           = 0;
			packet_sequence = 0;
			parity          = 0;
			_ischecksum     = false;
			sentence_type   = OTHER_SENTENCE;
			break;
						
			
			default:
			// all other characters
			if(index < sizeof(buffer)-1)
			buffer[index++] = c;
			
			if(!_ischecksum)
			parity ^= (uint8_t)c;
			break;
		}
	}
}

void GPSNMEA::process_packet()
{
	if(packet_sequence == 0)
	{
		if(strcmp(buffer, GPRMC) == 0){					
			sentence_type = GPRMC_SENTENCE;
			//Serial.println(buffer);
		}else if(strcmp(buffer, GPGGA) == 0){
			sentence_type = GPGGA_SENTENCE;
			//Serial.println(buffer);
		}else{
			sentence_type = OTHER_SENTENCE;
		}	
	}
	
	switch(COMBINE(sentence_type, packet_sequence))
	{
		case COMBINE(GPRMC_SENTENCE, 2):
		_sentence_has_fix = buffer[0] == 'A';
		break;
		
		case COMBINE(GPGGA_SENTENCE, 2):
		case COMBINE(GPRMC_SENTENCE, 3):
		ParseDegrees(buffer);
		//Serial.println(atol(buffer));
		// latitude
		break;
		
		case COMBINE(GPGGA_SENTENCE, 3):
		case COMBINE(GPRMC_SENTENCE, 4):
		// N|S
		break;
		
		case COMBINE(GPGGA_SENTENCE, 4):
		case COMBINE(GPRMC_SENTENCE, 5):
		//Serial.println(buffer);
		// Longitude
		break;
		
		case COMBINE(GPGGA_SENTENCE, 5):
		case COMBINE(GPRMC_SENTENCE, 6):
		// E|W
		break;
		
	    case COMBINE(GPRMC_SENTENCE, 7):
		// Speed in Knots
		break;
		
		case COMBINE(GPRMC_SENTENCE, 8):
		// COG in degrees
		break;
		
		case COMBINE(GPRMC_SENTENCE, 12):
		// Mode Indicator
		break;
		
		case COMBINE(GPGGA_SENTENCE, 6):
		// Position fix status : FS
		_valid_fix = buffer[0] != '0';
		// Serial.print(buffer[0]);Serial.print('\t');Serial.println(_valid_fix);
		break;
		
	    case COMBINE(GPGGA_SENTENCE, 7):
		// NoSatellites in Range  : NoSV
		break;
		
	   	case COMBINE(GPGGA_SENTENCE, 8):
		// Horizontal Dilution of Precision HDOP
		break;
	}
	
	if(_ischecksum)
	{
		byte checksum = 16 * fromHex(buffer[0]) + fromHex(buffer[1]);
		
		if(checksum == parity)
		{
			//Serial.print(parity); Serial.print('\t'); Serial.println(checksum);
		}
	}
}


int GPSNMEA::fromHex(const char a)
{
	if(a >= 'A' && a <= 'F')
		return a - 'A' + 10;
	else if(a >= 'a' && a <= 'f')
		return a - 'a' + 10;
	else
		return a - '0';
}

uint32_t GPSNMEA::ParseDegrees(char *buf)
{
	Serial.print(buf); Serial.println("Shiz");
	
	uint32_t leftOfdecimal  = (uint32_t)atol(buf) * 1e5;
	uint32_t  multiplier    = 1e5;
	uint32_t rightofdecimal = 0;
	while(isdigit(*buf)) *++buf;
	
	if(*buf == '.')
	{
		while(isdigit(*buf))
		{
			*++buf;
			Serial.print("What ");
			Serial.println(buf);
			
		}
	}
}

/*
$GPRMC,091009.00,A,0644.02644,S,038
57.21270,E,0.221,,2
90316,,,A*6F

$GPVTG,,T,,M,0.221,N,0.410,
K,A*27

$GPGGA,09
1009.00,0644.02644,S
,03857.21270,E,1,10,
0.76,124.5,M,-25.4,
M,,*61

$GPGSA,A,3,11
,17,06,23,07,03,28
,01,19,30,,,1.61,0
.76,1.41*04

$GPGSV,4,1,13,01,08,139,28
,02,01,280,,03,34,145,
19,06,28,246,31*
7D

$GPGSV,4,2,13,07
,18,359,27,08,08,07
0,20,09,31,033,26,1
1,10,119,21*78

$GPGSV,4,3,13,17,28,191
,22,19,10,216,30,23
,28,069,08,28,74,235
,09*7E

$GPGSV,4,4,
13,30,26,329,27*41$G
PGLL,0644.02644,S
,03857.21270,E,091
009.00,A,A*78

 */