#include "sdlib.h"

bool SD_Lib::initialise(HardwareSerial *Port, uint8_t resetOpenLog)
{
	_port = Port;
	if (setupOpenLog(Port, resetOpenLog))
	{
			return true;
	}
	else
	{
			return false;
	}
	/*
	if (setupOpenLog(Port, resetOpenLog) && 
		commandMode(*Port)				 &&
		changeDir(*Port, "TLOGS"))
		{
			return true;
		}
	else
	{
			return false;
	}
	*/
		
}

bool SD_Lib::setupOpenLog(HardwareSerial *Port, uint8_t resetOpenLog, uint32_t baud)
{
  pinMode(resetOpenLog, OUTPUT);
  Port->begin(baud);

  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);
  uint32_t tnow = millis();
  bool success = false;
   
  while(1) {
    if(Port->available())
      if(Port->read() == '<') {
        success = true;
        break;
      }
	uint32_t dt = millis()-tnow;
	
	if (dt > 1000)
	{
		Serial.println("sdlib: Failed to setup OpenLog");
		break;
	}
  }
  return success;	
}

bool SD_Lib::setupOpenLog(HardwareSerial *Port, uint8_t resetOpenLog)
{
  pinMode(resetOpenLog, OUTPUT);
  Port->begin(57600);

  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);
  uint32_t tnow = millis();
  bool success = false;
   
  while(1) {
    if(Port->available())
      if(Port->read() == '<') {
        success = true;
        break;
      }
	uint32_t dt = millis()-tnow;
	
	if (dt > 1000)
	{
		Serial.println("sdlib: Failed to setup OpenLog");
		break;
	}
  }
  return success;	
}

bool SD_Lib::commandMode(/*HardwareSerial  &Port*/)
{
  (*_port).write(26);
  (*_port).write(26);
  (*_port).write(26);  
  
  //Wait for OpenLog to respond with '>' to indicate we are in command mode
  bool success = false;
  uint32_t tnow = millis();
  while(1) {
    if((*_port).available())
      if((*_port).read() == '>'){
        success = true;
        break;
      }
	  
	uint32_t dt = millis()-tnow;
	if (dt > 1000)
	{
		Serial.println("sdlib: Failed to Enter Command Mode");
		break;  
	}
  }
  return success;
}


bool SD_Lib::changeDir(HardwareSerial &Port, char* dir_name)
{
	// consider checking whether dirname exists
  char str[80];
  strcpy(str, "cd ");
  strcat(str, dir_name);
  uint8_t len = strlen(str);
  Port.write(str,len);
  Port.write(13);     // carriage return needed - terminates command

  bool success = false;
  uint32_t tnow = millis();
  
  while(1)
  {
    if(Port.available())
    {
      char char_ = Port.read();
      if(char_ == '>')
      {
        success = true;
        break;
      }
    }
	
	uint32_t dt = millis()-tnow;
	if (dt > 1000)
	{
		Serial.println("sdlib: Failed to change Directory");
		break;  
	}
  }
  return success;	
}


void SD_Lib::write(uint8_t *buf, uint8_t len)
{
	_port->write(buf,len);
}


void SD_Lib::readOpenLog()
{
  char  msg_incoming[200];
  while((*_port).available())
  {
    char byte_ = (*_port).read();
    Serial.write(byte_);
  }  
}

bool SD_Lib::command(char* cmd ,char* arg, char cmp){
  char str[80];
  strcpy(str, cmd);
  strcat(str, " ");
  strcat(str, arg);
  uint8_t len = strlen(str);
  (*_port).write(str,len);
  (*_port).write(13);     // carriage return needed - terminates command

  bool success = false;
  while(1)
  {
    if((*_port).available())
    {
      char char_ = (*_port).read();
      if(char_ == cmp)
      {
        success = true;
        break;
      }
    }
  }
  return success;
}

bool SD_Lib::sendRaw(char* msg)
{
	(*_port).println(msg);
}