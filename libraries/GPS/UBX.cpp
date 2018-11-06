/*
    UBX.cpp - Ublox GNSS receiver interface library
    Copyright (C) 2016  Hery A Mwenegoha. All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>
 */

#include "UBX.h"

GPSUBX::GPSUBX(){	
	 _byPass_Header = false;	
	 _count		    = 0;
	 PAX_LENGTH     = 0;
	 FULL_LENGTH    = 0;
	 STOP_LENGTH    = 0;
	 SENTENCE_TYPE  = OTHER_UBX;
	 BaudRate       = 0;
	 
	 UBX.commit     = false;
	 UBX.gpsFixOk   = false;
}

// Get length of payload
uint8_t GPSUBX::GET_LENGTH(uint8_t class_id, uint8_t msg_id)
{
	// Just extend the message fields to receive other navigation messages
	if(class_id == 0x01)
	{
		switch(msg_id)
		{
			case 0x02: 
			SENTENCE_TYPE = NAV_POSLLH;
			return 28;
			
			case 0x03: 
			SENTENCE_TYPE = NAV_STATUS;
			return 16;
			
			case 0x06: 
			SENTENCE_TYPE = NAV_SOL;
			return 52;
			
			case 0x12:
			SENTENCE_TYPE = NAV_VELNED;
			return 36;
			
			case 0x04:
			SENTENCE_TYPE = NAV_DOP;
			return 18;
		}
	}
	else if(class_id == 0x06)
	{
		switch(msg_id)
		{
			case 0x00:
			SENTENCE_TYPE = CFG_PRT_UART;
			return 20;
			
			case 0x08:
			SENTENCE_TYPE = CFG_RATE_MEAS;
			return 6;
			
			case 0x01:
			SENTENCE_TYPE = CFG_MSG_RATE;
			return 3;
			
		}
	}
}

void GPSUBX::populate_buffer(byte *buffer1, byte *buffer2, uint8_t len)
{
	for(int i = 6; i < (len-2); i++)
	{
		buffer1[i-6] = buffer2[i];
	}
}

void GPSUBX::process_buffer(byte *buffer, uint8_t len)
{
	// start with fletcher checksum calculation..
	uint8_t CK_A = 0; uint8_t CK_B = 0;

	for(int i = 2; i < (len -2); i++)
	{
		CK_A  = CK_A + buffer[i];
		CK_B  = CK_B + CK_A;					
	}
	
	if(CK_A == buffer[len-2] && CK_B == buffer[len-1])
	{
		switch(SENTENCE_TYPE)
		{
			case CFG_PRT_UART:
			{
				_cfg_prt cfg_prt;		
				populate_buffer(cfg_prt.buffer, buffer, len);			
				BaudRate = cfg_prt.uart.baudrate;			
				// Serial.print("Baudrate: "); Serial.println(cfg_prt.uart.baudrate);
			}
			break;
			
			case CFG_RATE_MEAS:
			{
				_cfg_meas cfg_meas;
				populate_buffer(cfg_meas.buffer, buffer, len);
				
				#if PRINT_RATES
				Serial.print("measRate "); Serial.print(cfg_meas.packet.measRate); Serial.print('\t');
				Serial.print("navRate ");  Serial.print(cfg_meas.packet.navRate);  Serial.print('\t');
				Serial.print("timeRef ");  Serial.println(cfg_meas.packet.timeRef);
				#endif
			}
			break;
			
			case CFG_MSG_RATE:
			{
			}
			break;
			
			case NAV_POSLLH:
			{
				_nav_pos nav_pos;
				populate_buffer(nav_pos.buffer, buffer, len);				
				
				float hours        = ((float)nav_pos.packet.iTow/3600000);			
				uint8_t day        =  hours / 24;
				float hour_now     =  hours - 24  * day;
				float minutes_now  = (hour_now - (uint8_t)hour_now)*60;
				float seconds_now  = (minutes_now - (uint8_t)minutes_now) * 60;		
				
				if(UBX.commit)
				{
					commit_posllh(nav_pos.packet);
					
					UBX.hour    = (uint8_t)hour_now;
					UBX.minutes = (uint8_t)minutes_now;
					UBX.seconds = (uint8_t)seconds_now;
					
					/*
					#if PRINT_NAV_POSLLH 
					Serial.print("NAV_POSLLH: "); 
					Serial.print(UBX.lat,7);    Serial.print('\t');
					Serial.print(UBX.lon,7);    Serial.print('\t');
					Serial.print(UBX.height,2); Serial.print('\t');
					Serial.print(UBX.hMSL,2);   Serial.print('\t');
					
					Serial.print(UTC.hour);     Serial.print(":");
					Serial.print(UTC.minutes);  Serial.print(":");
					Serial.println(UTC.seconds);
					#endif
					*/
				}
			}
			break;
				
			case NAV_VELNED:
			{
				_nav_velned nav_velned;
				populate_buffer(nav_velned.buffer, buffer, len);
				
				if(UBX.commit)
				{
					commit_velned(nav_velned.packet);
					/*
					#if PRINT_NAV_VELNED					
					Serial.print("NAV_VELNED ");	
					Serial.print(UBX.velN,2);    Serial.print('\t');
					Serial.print(UBX.velE,2);    Serial.print('\t');
					Serial.print(UBX.velD,2);    Serial.print('\t');
					Serial.print("Speed ");
					Serial.print(UBX.gSpeed,2);  Serial.print('\t');
					Serial.print("Heading "); // most properly, course
					Serial.println(UBX.heading); 			
					#endif
					*/
				}
			}
			break;
			
			case NAV_SOL:
			{
				_nav_sol nav_sol;
				populate_buffer(nav_sol.buffer, buffer, len);			
				UBX.numSV = nav_sol.packet.numSV;
				
				#if PRINT_NAV_SOL
				Serial.print("NAV_SOL ");Serial.println(UBX.numSV);
				#endif
			}
			break;
			
			case NAV_STATUS:
			{
				_nav_status nav_status;
				populate_buffer(nav_status.buffer, buffer, len);
				byte mask    = B00000001;
				mask         =  mask & nav_status.packet.flags;
				UBX.gpsFixOk = mask == 1;
				UBX.fix_type = nav_status.packet.gpsFix; 
				UBX.commit   = UBX.fix_type != 0;
				
				if(UBX.commit)
					UBX.fix_time_ms = millis(); // the system millis that we have registered atleast a 1-DR fix.
				/*
				#if PRINT_STATUS
				Serial.print("iTow "); Serial.print(nav_status.packet.iTow);
				Serial.print(" Fix "); Serial.print(nav_status.packet.gpsFix);
				Serial.print(" Flag "); Serial.println(mask);
				#endif
				*/
			}
			break;
			
			case NAV_DOP:
			{
				_nav_dop nav_dop;
				populate_buffer(nav_dop.buffer, buffer, len);
				DOP.hDOP = nav_dop.packet.hDOP;
				DOP.vDOP = nav_dop.packet.vDOP;
				DOP.nDOP = nav_dop.packet.nDOP;
				DOP.eDOP = nav_dop.packet.eDOP;
				DOP.tDOP = nav_dop.packet.tDOP;
				
				/*
				#if PRINT_DOP
				Serial.print("NAV_DOP ");
				Serial.print(DOP.hDOP); Serial.print('\t');
				Serial.print(DOP.vDOP); Serial.print('\t');
				Serial.print(DOP.nDOP); Serial.print('\t');
				Serial.println(DOP.eDOP); 
				#endif
				*/
			}
			break;
			
		}
	}
}

void GPSUBX::UBX_decode(uint8_t read_byte, byte *buffer)
{
    // Serial.println(read_byte);
	// byte buffer[58] passed as largest navigation buffer possible -> see UBX Class NAV
	if(!_byPass_Header)
	{
		if(read_byte == 0xB5)
		{
			_count 			= 0; 
			buffer[_count] 	= read_byte;
		}		
		else if(read_byte == 0x62)
		{
			_count++; 
			if(_count == 1){
				_byPass_Header  = true;
				buffer[_count]  = read_byte; 
			}
		}
	 }
	 else
	 {
		_count++;
		buffer[_count] = read_byte;
	
		if(_count == 5)
		{
			// check the message length and calculate full length
			_I16_un _I16;
			_I16.buffer[0] 	  = buffer[4];
			_I16.buffer[1] 	  = buffer[5];
			
			uint8_t class_id  = buffer[2];
			uint8_t msg_id    = buffer[3];
			
			bool MATCH = GET_LENGTH(class_id, msg_id) == _I16.value;
			
			if(MATCH)
			{
				PAX_LENGTH  = GET_LENGTH(class_id, msg_id);
				
				FULL_LENGTH = PAX_LENGTH + 8;
				
				STOP_LENGTH = FULL_LENGTH - 1;
			}
			else
			{
				_byPass_Header = false;
			}
		}
		else if(_count == STOP_LENGTH)
		{
			_byPass_Header = false;
			
			// Process buffer:
			process_buffer(buffer, FULL_LENGTH);
		}		
	 }		
}

void GPSUBX::configure_measurementRate()
{
	byte CK_A = 0;
	byte CK_B = 0;
	byte cgf_rate_buffer[] = {0xB5, 0x62, 0x06, 0x08, 6, 0, 200,0, 1,0,1,0, CK_A, CK_B};
	byte len = sizeof(cgf_rate_buffer);
	calculate_checksum(cgf_rate_buffer, len);	
	Port->write(cgf_rate_buffer, len);
	delay(200);
}

void GPSUBX::request_measurementRate()
{
	byte CK_A = 0;
	byte CK_B = 0;
	byte cgf_rate_buffer[] = {0xB5, 0x62, 0x06, 0x08, 0, 0, CK_A, CK_B};
	byte len = sizeof(cgf_rate_buffer);
	calculate_checksum(cgf_rate_buffer, len);	
	Port->write(cgf_rate_buffer, len);
	delay(200);
}

void GPSUBX::request_msgConfig(uint8_t msg_class, uint8_t msg_id)
{
	byte CK_A = 0;
	byte CK_B = 0;
	byte cgf_rate_buffer[] = {0xB5, 0x62, 0x06, 0x01, 2, 0, msg_class, msg_id, CK_A, CK_B};
	byte len = sizeof(cgf_rate_buffer);
	calculate_checksum(cgf_rate_buffer, len);	
	Port->write(cgf_rate_buffer, len);
	delay(200);	
}


bool GPSUBX::initialise(HardwareSerial *_Port)
{
	Port = _Port;
	
	// Poll CFG_PRT to get configuration message -> remember length is 2 bytes
	byte CK_A = 0;
	byte CK_B = 0;
	byte buffer[] = {0xB5, 0x62, 0x06, 0x00, 0, 0, CK_A, CK_B};
	byte len = sizeof(buffer);
		
	calculate_checksum(buffer, len);	
		
	uint32_t baud[] = {4800, 9600, 19200, 38400, 57600, 115200};
	
	// blocking code..
	byte counter = 0;
	while(BaudRate == 0 && counter < 4)
	{	
		for(int i = 0; i < 6; i++)
		{
			Port->begin(baud[i]);
			
			Port->write(buffer, len);
			
			delay(200);
					
			byte buffer[UBX_BUFFER_SIZE];
			while(Port->available())
			{			
				byte read_byte = Port->read();				
				UBX_decode(read_byte, buffer);		
			}
		}
		counter++;
	}
	
	Port->begin(BaudRate);
	
	if(BaudRate == 0)
	{
		return false;
	}
	else
	{
		configure_measurementRate(); // 5Hz update rate setup
		UBX.fix_time_ms = millis();
		update_Msec  = UBX.fix_time_ms;
		return true;
	}
	
	//UBX.fix_time_ms = millis();
	//update_Msec  = UBX.fix_time_ms;
}



bool GPSUBX::initialise(HardwareSerial *_Port, uint32_t baud)
{
	Port = _Port;
	BaudRate = baud;
	Port->begin(BaudRate);
	UBX.fix_time_ms = millis();
	update_Msec  = UBX.fix_time_ms;
	return true;
}



void GPSUBX::calculate_checksum(byte *buffer, const byte len)
{
	byte CK_A = 0;
	byte CK_B = 0;
	
	for(int i = 2; i < len-2; i++)
	{
		CK_A = CK_A + buffer[i];
		CK_B = CK_B + CK_A;
	}
	
	buffer[len-2] = CK_A;
	buffer[len-1] = CK_B;
}

bool GPSUBX::process_stream()
{	
	uint8_t buffer[UBX_BUFFER_SIZE];
	while(Port->available())
	{
		uint8_t read_byte = Port->read();	
		UBX_decode(read_byte, buffer);
	}
	//uint32_t t_now = millis();
	
	if(UBX.fix_time_ms != update_Msec)
	{
		uint32_t lapse = UBX.fix_time_ms - update_Msec;
		update_Msec    = UBX.fix_time_ms;
		return true;
		/*
		Serial.print("GPS ");
		Serial.print("	");
		Serial.println(lapse);
		*/
	}
	else
		return false;
}

uint32_t GPSUBX::get_BaudRate()
{
	return BaudRate;
}
