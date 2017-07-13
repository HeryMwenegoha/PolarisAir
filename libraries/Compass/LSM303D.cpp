#include "LSM303D.h"
#include "Arduino.h"

//AP_Compass_LSM303D AP_compass_lsm303d;

#define ADDRESS	0x1E

#define CRA_REG_M			0x00
#define CRA_REG_ODR_15HZ	B00010000

#define CRB_REG_M	0x01
//#define CRB_REG_GN_1_9	B01000000
#define CRB_REG_GN_1_3	B00100000

#define MR_REG_M	0x02
#define MR_REG_CONTINOUS	B00000000

#define OUT_X_H_M	 0x03
#define OUT_TEMP_H_M 0x31
#define OUT_TEMP_L_M 0x32

#define SR_REG_M	0x09


void  AP_LSM303D::initialise()
{
	// data rate set to 15Hz
	Wire.write8(ADDRESS, CRA_REG_M, (byte)(CRA_REG_ODR_15HZ));
	delay(10);
	Wire.write8(ADDRESS, CRA_REG_M, (byte)(CRA_REG_ODR_15HZ));
	delay(10);
	if(Wire.read8(ADDRESS, CRA_REG_M) != (byte)(CRA_REG_ODR_15HZ)){
		_have_sens = false;
		return;
	}
	delay(10);
	
	// setup the gain 1.9 XYZ 855 LSB/Gass; Z  -  760 LSB/Gauss
	// setup the gain 1.3 XYZ 1100 LSB/Gass; Z  -  980 LSB/Gauss
	Wire.write8(ADDRESS, CRB_REG_M, (byte)(CRB_REG_GN_1_3));
	delay(10);
	Wire.write8(ADDRESS, CRB_REG_M, (byte)(CRB_REG_GN_1_3));
	delay(10);
	
	// continous conversion
	Wire.write8(ADDRESS, MR_REG_M, (byte)(MR_REG_CONTINOUS));
	delay(10);
	Wire.write8(ADDRESS, MR_REG_M, (byte)(MR_REG_CONTINOUS));
	delay(10);
	
	_update_us    = micros();
	_last_time_us = micros();
	_have_sens	  = true;	
	
	AP_compass_backend.register_device(LSM303D);
	Serial.println(F("AP_LSM303D::	Compass	Ready"));
}

// configured to give values between +/- 1.3 Gauss range from -2048 - 2047 LSB
// Gain is 1100 for XY and 980 for Z.
void AP_LSM303D::accumulate()
{
	if(_have_sens == false) return;
	// 15hz updates
	uint64_t _time = micros();
	if(_time - _update_us >= 66666)
	{
		_update_us = _time;		
		const uint8_t _status = (Wire.read8(ADDRESS, SR_REG_M) & B00000001);		
		if(_status == 1)
		{
			//const uint8_t* buffer = Wire.readReg(ADDRESS, (uint8_t)(OUT_X_H_M | 0x80), 6);

			uint8_t buffer[6];
			
			Wire.beginTransmission(ADDRESS);
			Wire.write((OUT_X_H_M | (1 << 7)));
			Wire.endTransmission();
			
			Wire.requestFrom(ADDRESS, 6);
			uint32_t _start_msec = millis();
			while(Wire.available() < 6){
				
				if((millis() - _start_msec) >= 50){
					Serial.println("LSM303M	I2C	Timeout");
					return;
				}
				
			}			
			buffer[0] = Wire.read();
			buffer[1] = Wire.read();				
			buffer[2] = Wire.read();
			buffer[3] = Wire.read();	
			buffer[4] = Wire.read();
			buffer[5] = Wire.read();
			
			vector3f _mag_vector;
			_mag_vector.x  =   (int16_t)(buffer[0] << 8 | buffer[1]);
			_mag_vector.y  =  -(int16_t)(buffer[4] << 8 | buffer[5]); 
			_mag_vector.z  =  -(int16_t)(buffer[2] << 8 | buffer[3]); 												

			//_mag_vector.print(0);
			
			AP_compass_backend.filter_raw_sample(_mag_vector, 0.07);

			uint64_t time = micros();	
			_dt_mag = static_cast<float>(time - _last_time_us) * 1e-6f;
			_last_time_us = time;		
		}
	}
}


// Called from front end whenever new data is available
void AP_LSM303D::update()
{		
	if(_have_sens == false) return;
	_backend->update_magnetometer();
}

