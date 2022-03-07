#include "BMP180.h"

#define OVERSAMPLING 3
#define FILTER_SIZE	 10

SFE_BMP180::SFE_BMP180()
{
  STATE_MACHINE = 1;
}


bool SFE_BMP180::begin()
{
	STATE_MACHINE = 1;
	_state        = 0;	
	double c3,c4,b1;	

	
	if (readInt(0xAA,AC1) &&
		readInt(0xAC,AC2) &&
		readInt(0xAE,AC3) &&
		readUInt(0xB0,AC4) &&
		readUInt(0xB2,AC5) &&
		readUInt(0xB4,AC6) &&
		readInt(0xB6,VB1) &&
		readInt(0xB8,VB2) &&
		readInt(0xBA,MB) &&
		readInt(0xBC,MC) &&
		readInt(0xBE,MD))
	{
		
		c3 =  160.0		 	 * pow(2,-15) * AC3;
		c4 =  pow(10,-3)	 * pow(2,-15) * AC4;
		b1 =  pow(160,2)	 * pow(2,-30) * VB1;
		c5 = (pow(2,-15) / 160) * AC5;
		c6 =  AC6;
		mc = (pow(2,11) / pow(160,2)) * MC;
		md =  MD / 160.0;
		x0 =  AC1;
		x1 =  160.0 * pow(2,-13) * AC2;
		x2 =  pow(160,2) * pow(2,-25) * VB2;
		y0 =  c4 * pow(2,15);
		y1 =  c4 * c3;
		y2 =  c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 =  1.0 - 7357.0 * pow(2,-20);
		p2 =  3038.0 * 100.0 * pow(2,-36);


		// Establish BaselinePressure
		byte index_counter = 0;
		double  temp	   = 20; // initialised to room temperature
		for(int i = 0; i< 100; i++)
		{	
			startTemperature();
			delay(10);
			getTemperature(temp);
			index_counter++;
			
			if(index_counter >= 50){
				_temperature = 0.95 * temp + 0.05 * _temperature;
			}else{
				_temperature = temp;
			}
		}
		

	    char _delay = startPressure(OVERSAMPLING);
	    if(_delay == 0){
			Serial.println("BMP180	ERROR");
			return false;
		}
		delay(26);
	    getPressure(_raw_pressure,_temperature);

		baseline  	   = _raw_pressure;
		_avg_pressure  = baseline;	
		startTemperature();
		
		Serial.print("Baro");Serial.print("\t");Serial.print(_temperature);Serial.print("\t");Serial.println(baseline);
		
		_pressure_sum = 0;
		_count        = 0;
		_base_update  = false;
		return true;	
	}
	else
	{
		// Error reading calibration data; bad component or connection?
		return false;
	}
}

// Helper functions
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
char SFE_BMP180::readInt(char address, int16_t &value)
{
	unsigned char data[2];

	data[0] = address;
	if (readBytes(data,2))
	{
		value = (int16_t)((data[0]<<8)|data[1]);
		//if (*value & 0x8000) *value |= 0xFFFF0000; // sign extend if negative
		return(1);
	}
	value = 0;
	return(0);
}



// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
char SFE_BMP180::readUInt(char address, uint16_t &value)
{
	unsigned char data[2];

	data[0] = address;
	if (readBytes(data,2))
	{
		value = (((uint16_t)data[0]<<8)|(uint16_t)data[1]);
		return(1);
	}
	value = 0;
	return(0);
}


// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
char SFE_BMP180::readBytes(unsigned char *values, char length)
{
	char x;
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values[0]);
	_error = Wire.endTransmission();
	if (_error == 0)
	{
		Wire.requestFrom(BMP180_ADDR,length);
		int z = 0;
		while(Wire.available() != length)// wait until bytes are ready
		{
		  z++;
		  if(z >= 200)
		  {
			break;
			return(0);
		  }
		}
		for(x=0;	x<length;	x++)
		{
			values[x] = Wire.read();
		}
		return(1);
	}
	return(0);
}



// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
char SFE_BMP180::writeBytes(unsigned char *values, char length)
{
	char x;	
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values,length);
	_error = Wire.endTransmission();
	if (_error == 0)
		return(1);
	else
		return(0);
}



// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
char SFE_BMP180::startTemperature(void)
{
	unsigned char data[2], result;
	
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	result  = writeBytes(data, 2);
	if (result) // good write?
	{
		_last_cmd_temp_read_msec = millis();
		return(5); // return the delay in ms (rounded up) to wait before retrieving data
	}
	else
	{
		return(0); // or return 0 if there was a problem communicating with the BMP
	}
}





char SFE_BMP180::getTemperature(double &T)
{
	unsigned char data[2];
	char result;
	double tu, a;	
	data[0] = BMP180_REG_RESULT;
	result  = readBytes(data, 2);
	if (result) // good read, calculate temperature
	{
		tu = (data[0] * 256.0) + data[1];
		a = c5 * (tu - c6);
		T = a + (mc / (a + md));
	}
	return(result);
}


// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
char SFE_BMP180::startPressure(char oversampling)
{
	unsigned char data[2], result, delay;
	
	data[0] = BMP180_REG_CONTROL;

	switch (oversampling)
	{
		case 0:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
		case 1:
			data[1] = BMP180_COMMAND_PRESSURE1;
			delay = 8;
		break;
		case 2:
			data[1] = BMP180_COMMAND_PRESSURE2;
			delay = 14;
		break;
		case 3:
			data[1] = BMP180_COMMAND_PRESSURE3;
			delay = 26;
		break;
		default:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
	}
	result = writeBytes(data, 2);
	if (result) // good write?
	{
		_last_cmd_pres_read_msec = millis();
		return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	}
	else
	{
		return(0); // or return 0 if there was a problem communicating with the BMP
	}
}



char SFE_BMP180::getPressure(double &P, double &T)
{
	unsigned char data[3];
	char result;
	double pu,s,x,y,z;
	
	data[0] = BMP180_REG_RESULT;

	result = readBytes(data, 3);
	if (result) // good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);
		s = T - 25.0;
		x = (x2 * pow(s,2)) + (x1 * s) + x0;
		y = (y2 * pow(s,2)) + (y1 * s) + y0;
		z = (pu - x) / y;
		P = (p2 * pow(z,2)) + (p1 * z) + p0;
	}
	return(result);
}



char SFE_BMP180::getError(void)
{
	return(_error);
}


// Call this function first
float SFE_BMP180::getBaselinePressure()
{
  char status;
  double T,P;
  status   = startTemperature();
  if (status != 0)
  {
    delay(5);
    status = getTemperature(T);
    if (status != 0)
    {
      status = startPressure(3);
      if (status != 0)
      {      
        delay(26);
        status = getPressure(P,T);      
        if (status != 0)
        {
          baseline = P;
          return(P);
        }
        else Serial.println(F("error retrieving pressure measurement\n"));
      }
      else Serial.println(F("error starting pressure measurement\n"));
    }
    else Serial.println(F("error retrieving temperature measurement\n"));
  }
   else Serial.println(F("error starting temperature measurement\n"));  
}



bool SFE_BMP180::read()
{
  // check to see if data is ready
  if(!data_ready()){
	  return false;
  }
  
  if(_state == 0){
	  getTemperature(_temperature);
//_temperature = 29;
  }
  else if(getPressure(_raw_pressure,_temperature))
  {
	_count++;
	_pressure_sum += _raw_pressure;
	
	if(_count == FILTER_SIZE)
	{
		_avg_pressure = _pressure_sum/_count;
		_pressure_sum = 0;
		_count        = 0;
		#if DEBUG
		Serial.print((millis()-last_update_msec)*1e-3f, 3);
		Serial.print("\t");
		Serial.print(_temperature);
		Serial.print("\t");
		Serial.println(_avg_pressure);
		last_update_msec = millis();
		# endif		
	}
	
	
	last_update_msec = millis();	
  }

  _state++;
  
  if(_state == 25){
	  _state = 0;
	  startTemperature(); 
  }else{
	  startPressure(OVERSAMPLING);
  }

}

bool SFE_BMP180::data_ready(){
	
	if(_state == 0){
		return millis() > _last_cmd_temp_read_msec + 5;
	}
	
	byte conversion_time = 0;
	
	switch(OVERSAMPLING)
	{
		case 0:
			conversion_time = 5;
			break;
		
		case 1:
			conversion_time = 8;
			break;
		
		case 2:
			conversion_time = 14;
			break;
			
		case 3:
			conversion_time = 26;
			break;
	}
	
	return millis() > _last_cmd_pres_read_msec + conversion_time;
}


bool SFE_BMP180::temperature(float &temp)
{
	if((millis() - last_update_msec > 250))
		return false;
	temp = _temperature;
	return true;
}
  
 
bool SFE_BMP180::pressure(float  &press)
{
	if((millis() - last_update_msec > 250))
		return false;
	press = _avg_pressure;
	return true;
}


float SFE_BMP180::baseline_pressure()
{
	return baseline;
}
