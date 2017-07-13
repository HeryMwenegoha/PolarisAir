#pragma once
/*
 * By Hery A Mwenegoha Copyright 2017
 * NMEA GPS Module
 */
class BeStar
{
	BeStar():
	next_char(0),
	parity(1),
	parity_stop(false)
	{
		
	}
	//char GPRMC[] = "$GPRMC";
	//char GPGGA[] = "$GPGGA";
	uint8_t next_char;
	uint8_t parity;
	bool    parity_stop;
    void 	read();
	uint8_t hex2dec(char _char);
	uint8_t parser(char c, char *buffer, char *sentence);
	float   convert2Deg(char* input, char *dir);
	
	~BeStar(){
		
	}
};