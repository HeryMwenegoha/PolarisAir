#include "MS4525DO.h"
#include "Wire.h"

#define ADDRESS1 0x28
//#define ADDRESS2 0x36
//#define ADDRESS3 0x46

#define Read_DF4 0x01  		// long fetch

AP_MS4525DO AP_ms4525do;	// singleton class

bool AP_MS4525DO::init()
{	
	startmeasurement();
	delay(40);
	bool ret = collectdata();
	delay(20);
	return ret;
}

// mainloop
void AP_MS4525DO::startmeasurement()
{
	Wire.beginTransmission(ADDRESS1);
	Wire.write(Read_DF4);
	Wire.endTransmission();
}

// mainloop
bool AP_MS4525DO::collectdata()
{
	byte buffer[4];
    byte status_byte = 3;
	Wire.requestFrom(ADDRESS1, 4);
	if(Wire.available() >= 4)
	{
		buffer[0]    	= Wire.read();
		buffer[1]    	= Wire.read();
		buffer[2]    	= Wire.read();
		buffer[3]    	= Wire.read();	
		
		status_byte  	= ((buffer[0] & 0xC0) >> 6);						
	}
	
	int   dT_raw, dP_raw;
	
	if(status_byte == 0)
	{
		dP_raw = (buffer[0]<< 8) + buffer[1]; 	                    
		dP_raw = (0x3FFF & dP_raw) ; 	                    				 					 // Mask the 2 MSBs to get 14Bit Precision				
		float diff_pressure = ((dP_raw - 0.1f * 16383) * (Pmax - Pmin)/(0.8f * 16383)) + Pmin;   // PSI pressure as is from the tube.
		pressure = diff_pressure;
				
		dT_raw   = (buffer[2] << 8) + buffer[3];   
		dT_raw   = (0xFFE0 & dT_raw) >> 5;                      	   // we dont want the 5 LSB - shift by 5.
		temperature 	 = ((dT_raw * 200.0f)/2047.0f) - 50.0f;		   // Celsius
		last_sample_time_msec = millis();
		return true;
	}
	else
	{
		return false;
	}
}

void AP_MS4525DO::measure()
{
	// try take semaphore and block for 1ms
	if(millis() - last_Msec >= 10)  // 40ms btn reads
	{
		//Serial.println(millis() - last_Msec);
		last_Msec = millis();
		
		switch(Do_Event)
		{
			case 1:
			Do_Event++;
			startmeasurement();
			break;

			case 2:
			Do_Event--;
			collectdata();
			break;
		}
    }
	
}

void AP_MS4525DO::measure(byte del_msec)
{
	startmeasurement();
	delay(del_msec);
	collectdata();
}

bool AP_MS4525DO::get_differential_pressure(float &_press)
{
	if(millis() - last_sample_time_msec > 100)
	{
		return false;
	}
	_press =  pressure; // PSI
	return true;
}

bool AP_MS4525DO::get_temperature(float &_temp)
{
	if(millis() - last_sample_time_msec > 100)
	{
		return false;
	}
	_temp = temperature; // Celsius
	return true;
}





