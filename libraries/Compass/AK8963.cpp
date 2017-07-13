#include "AK8963.h"
#include "Arduino.h"

#define ADDRESS				0x0C 
#define WIA_REG				0x00
#define DEVICE_ID			0x48
#define I2C_MST_CTRL_REG	0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define EXT_SENS_DATA_00	0x49
#define I2C_SLV0_DO			0x63
#define STATUS1_REG			0x02
#define STATUS2_REG			0x09  		// Talks about sensor overflow
#define CNTL_REG 			0x0A
#define CNTL_SET			B00010010   // 16 Bit 8Hz Continous Mode1
#define ASAX_REG			0x10	
#define ASAY_REG			0x11
#define ASAZ_REG			0x12		
#define HXL_REG				0x03
#define HXH_REG				0x04
#define HYL_REG				0x05
#define HYH_REG				0x06
#define HZL_REG				0x07
#define HZH_REG				0x08


void  AP_AK8963::initialise()
{	
	// Confirm ID, if nothing move on dont waste time
	if(Wire.read8(ADDRESS, WIA_REG) != DEVICE_ID){
		_have_sens = false;
		return;
	}
	delay(5);

	// Power down Magnetometer
	Wire.write8(ADDRESS, CNTL_REG, 0x00);
	delay(15);
	
	// FuseRom Access Mode
	Wire.write8(ADDRESS, CNTL_REG, 0x0F);
    delay(15);
	
	// Read sensitivity Adjustment values from Fuse Rom since we are in Fuse ROM Mode
	uint8_t sensitivity[3];
	Wire.readBytes(ADDRESS, ASAX_REG, 3, &sensitivity[0]);
	ASA.x = (float)(sensitivity[0] - 128)/256. + 1.;
	ASA.y = (float)(sensitivity[1] - 128)/256. + 1.;
	ASA.z = (float)(sensitivity[2] - 128)/256. + 1.;
	
	ASA.print();
	
	// Power down after FuseROM access [Recommended in AK8963 Document]
	Wire.write8(ADDRESS, CNTL_REG, 0x00);
	delay(15);
	
	// Continous Read Mode with 16Bit Resolution
	Wire.write8(ADDRESS, CNTL_REG, CNTL_SET);
	delay(15);
		
	_update_us    = micros();
	_last_time_us = micros();
	Serial.println(F("AP_AK8963::	Compass	Ready"));
	_have_sens = true;	
	AP_compass_backend.register_device(AK8963);
}


void AP_AK8963::accumulate()
{
	if(_have_sens == false) return;
	
	uint64_t _time = micros();
	if(_time - _update_us >= 125000)
	{
		_update_us = _time;	
		
		uint8_t buffer[7];
		vector3f _mag_vector	 = vector3f(0,0,0);
		const uint8_t _dataready = (Wire.read8(ADDRESS, STATUS1_REG) & B00000001);
		
		if(_dataready == 1)
		{
			Wire.readBytes(ADDRESS, HXL_REG, 7, &buffer[0]);			
			const uint8_t _overflow  = (buffer[6] & B00001000) >> 3;
			if(_overflow == false)
			{				
				_mag_vector.x = (int16_t)(buffer[1] << 8 | buffer[0]);
				_mag_vector.y = (int16_t)(buffer[3] << 8 | buffer[2]); 
				_mag_vector.z = (int16_t)(buffer[5] << 8 | buffer[4]);	
								
				_mag_vector.x = _mag_vector.x * ASA.x;
				_mag_vector.y = _mag_vector.y * ASA.y;
				_mag_vector.z = _mag_vector.z * ASA.z;
				
				_mag_vector.x = (int16_t)_mag_vector.x;
				_mag_vector.y = (int16_t)_mag_vector.y;
				_mag_vector.z = (int16_t)_mag_vector.z;
				
				/*
				determine_max(_mag_vector, _offset.max);
				determine_min(_mag_vector, _offset.min);
				_offset.update();
				*/
				
				_mag_vector = _mag_vector - _offset.main;
				
				//_mag_vector.print(0);
				
				
				if(_dt_mag > 1.0f)
					_dt_mag = 0.0f;
						
				AP_compass_backend.filter_raw_sample(_mag_vector, _dt_mag);		

				uint64_t time = micros();	
				_dt_mag 	  = static_cast<float>(time - _last_time_us) * 1e-6f;
				_last_time_us = time;		 
			}
		}
	}
}

void AP_AK8963::determine_max(vector3f &value1, vector3f &value2){
	
	bool trigger = false;
	
	if(value1.x > value2.x){
		value2.x = value1.x;
		trigger = true;
	}
	
	if(value1.y > value2.y){
		value2.y = value1.y;
		trigger = true;
	}
	
	if(value1.z > value2.z){
		value2.z = value1.z;
		trigger = true;
	}
	
	if(trigger){
		//Serial.print(F("Max:	"));
		//value2.print();
	}
}

void AP_AK8963::determine_min(vector3f &value1, vector3f &value2){
	bool trigger = false;
	
	if(value1.x < value2.x){
		value2.x = value1.x;
		trigger = true;
	}
	
	if(value1.y < value2.y){
		value2.y = value1.y;
		trigger = true;
	}
	
	if(value1.z < value2.z){
		value2.z = value1.z;
		trigger = true;
	}
	
	if(trigger){
		//Serial.print(F("Min:	"));
		//value2.print();
	}
}


// Called from front end whenever new data is available
void AP_AK8963::update()
{		
	if(_have_sens == false) return;
	_backend->update_magnetometer();
}
