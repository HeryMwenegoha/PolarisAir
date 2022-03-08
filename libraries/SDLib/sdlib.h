#pragma once
/*
 * BY Hery A Mwenegoha (C) 2019
 * Log data into the open log for post processing
 * Serial3 is dedicated for OpenLog
 */
#include "Arduino.h"
class SD_Lib
{
	public:
	SD_Lib()
	{
	};
	bool initialise(HardwareSerial *Port, uint8_t resetPin);
	bool initialise(HardwareSerial *Port, uint8_t resetPin, uint32_t baud);
	void write(uint8_t *buf, uint8_t len);
	
	void readOpenLog();
	bool command(char* cmd ,char* arg, char cmp);
	
	bool commandMode();
	bool sendRaw(char* msg);
	
	private:
	bool setupOpenLog(HardwareSerial  *Port, uint8_t resetOpenLog);
	//bool commandMode(HardwareSerial  &Port);
	bool changeDir(HardwareSerial &Port, char* dir_name);
	
	HardwareSerial *_port;
};