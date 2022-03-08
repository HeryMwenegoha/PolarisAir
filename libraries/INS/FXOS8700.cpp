/* (C) Hery A Mwenegoha UoN 2019
*/
#include "FXOS8700.h"
#define ADRESS		0x1F 
#define PRODUCT_ID  0xC7
#define WHO_AM_I	0x0D


/* CTRL_REG1	0x2A
 * Item		Bit 	Bit-Value	Effect-Value
 * aslp	   	[7:6]	00			auto wake sample frequency 50Hz
 * dr		[5:3]	010			ODR @ Hybrid Mode 100Hz
 * lnoise	[2]		0			Normal Model
 * F_read	[1]		0			Fast_Read = Normal
 * ACTIVE 	[0]		0			Active=Standby Mode
 */
#define CTRL_REG1				 0x2A
#define CTRL_REG1_SETUP			 (0x02 << 3) 
#define CTRL_REG1_ACTIVE	     (0x02 << 3) | (0x01)
#define CTRL_REG1_STANDBY	      0x00							


/* CTRL_REG2	0x14  
 * Item				Bit 	Bit-Value	Effect-Value
 * st	   	[7]		0			To INT2
 * rst		[6]		0			FIFO INT DIS
 * smods		[5]		0			TO INT2
 * slpe		[4]		0			RT INT DIS
 * mods 	[3]		0			To INT2 
 * INT_EN_DRDY 		[2]		0			DR INT DIS
 * IPO_L			[1]		0			ACTIVE LOW
 * PP_O_D			[0]		0			PUSH-PULL
 */
#define CTRL_REG2				 0x2B



#define XYZ_DATA_CFG 		 0x0E
#define XYZ_DATA_CFG_SETUP 	 0x01

#define HP_FILTER_CUTOFF     	0x0F
#define HP_FILTER_CUTOFF_SETUP (0x01 << 5)


#define M_DR_STATUS  0x32
#define OUT_M_X_H    0x33
#define OUT_M_X_L	 0x34
#define OUT_M_Y_H	 0x35
#define OUT_M_Y_L	 0x36
#define OUT_M_Z_H	 0x37
#define OUT_M_Z_L	 0x38
#define M_CTRL_REG1  0x5B
#define M_CTRL_REG2  0x5C


/* TEMP 0x12
 * Range -40 +125
 */
#define OUT_TEMP	0x51

#define STATUS_REG	0x00  
#define OUT_X_H		0x01
#define OUT_X_L		0x02
#define OUT_Y_H		0x03
#define OUT_Y_L		0x04
#define OUT_Z_H		0x05
#define OUT_Z_L		0x06


AP_FXOS8700 *AP_FXOS8700::instance = NULL;


bool AP_FXOS8700::initialise()
{
	if(PRODUCT_ID != Wire.read8(ADRESS, WHO_AM_I, false)){
		_have_sens = false;
		return false;
		
	}
	delay(10);
	
	// Device on Standby
	Wire.write8(ADRESS, CTRL_REG1, (byte)CTRL_REG1_STANDBY);
	delay(15);	
	Wire.write8(ADRESS, CTRL_REG1, (byte)CTRL_REG1_STANDBY);
	delay(15);

	
	// FS selection : +/-4g
	Wire.write8(ADRESS, XYZ_DATA_CFG, (byte)XYZ_DATA_CFG_SETUP);		
	delay(15);	
	Wire.write8(ADRESS, XYZ_DATA_CFG, (byte)XYZ_DATA_CFG_SETUP);		
	delay(15);
	
	
	// High resolution
	Wire.write8(ADRESS, CTRL_REG2, (byte)0x02);		
	delay(15);	
	Wire.write8(ADRESS, CTRL_REG2, (byte)0x02);		
	delay(15);
		
	
	// 100Hz ODR Hybrid , Low Noise
	Wire.write8(ADRESS, CTRL_REG1, (byte)0x15);		
	delay(15);	
	Wire.write8(ADRESS, CTRL_REG1, (byte)0x15);		
	delay(15);
	
	// Magnetometer : Hybrid OSR=16
	Wire.write8(ADRESS, M_CTRL_REG1, (byte)0x1F);		
	delay(15);	
	Wire.write8(ADRESS, M_CTRL_REG1, (byte)0x1F);		
	delay(15);
	
	// Magnetometer : skip to 0x33 after 0x06
	Wire.write8(ADRESS, M_CTRL_REG2, (byte)0x20);		
	delay(15);	
	Wire.write8(ADRESS, M_CTRL_REG2, (byte)0x20);		
	delay(15);
	
	//_imu->register_device(PRODUCT_ID, 760);
	
	_last_update_us = micros();
	_update_us      = micros();
	Serial.println(F("AP_FX0S8700::	Accel & Compass	Ready"));
	_have_sens = true;
	return true;	
}


void AP_FXOS8700::accumulate()
{
	// gather data and update backend with new filtered data info
	if(!_have_sens)	return;
	
 	uint64_t _time = micros();
	if(_time - _update_us >= 10000)
	{
		_update_us = _time;
		uint8_t buffer[12];	
		
		Wire.beginTransmission(ADRESS);
		Wire.write((STATUS_REG | 0x80));
		Wire.endTransmission();
		
		Wire.requestFrom((byte)ADRESS, (byte)13);
	    uint32_t _start_msec = millis();
		while(Wire.available() < 13){
			if((millis() - _start_msec) >= 50){
				Serial.println(F("AP_FXOS8700	I2C	Timeout"));
				return;
			}
		}
		
		const uint8_t _status = ((Wire.read() & 0x08) >> 3);
		
		if(_status == 1)
		{					
			buffer[0] = Wire.read();
			buffer[1] = Wire.read();				
			buffer[2] = Wire.read();
			buffer[3] = Wire.read();	
			buffer[4] = Wire.read();
			buffer[5] = Wire.read();	
			
			buffer[6] = Wire.read();
			buffer[7] = Wire.read();				
			buffer[8] = Wire.read();
			buffer[9] = Wire.read();	
			buffer[10] = Wire.read();
			buffer[11] = Wire.read();
			
			vector3f _accel_vector;
			vector3f _mag_vector;
			
			_accel_vector.x =  (int16_t)(buffer[0] << 8 | buffer[1]) >> 2;
			_accel_vector.y = -(int16_t)(buffer[2] << 8 | buffer[3]) >> 2;
			_accel_vector.z = -(int16_t)(buffer[4] << 8 | buffer[5]) >> 2;
			
		    _mag_vector.x =  (int16_t)((buffer[6]  << 8)  | buffer[7]);
			_mag_vector.y = -(int16_t)((buffer[8]  << 8)  | buffer[9]);
			_mag_vector.z = -(int16_t)((buffer[10] << 8)  | buffer[11]);
				
			if(_dt_gyro > 1.0f){
				_dt_gyro = 0.01f;
			}
			

			vector3f mag_offset_max = vector3f( 967,  -333,  -292); // LSB vector3f(96.70,  -33.30,  -29.20);   // uT  
		    vector3f mag_offset_min = vector3f(-520, -1302, -1223); // LSB vector3f(-5.20, -130.20, -122.30);   // uT  
			vector3f mag_offset     = (mag_offset_max + mag_offset_min)/2; // uT
			
			vector3f _mag_field;
			_mag_field.x 	  = (_mag_vector.x - mag_offset.x);
			_mag_field.y 	  = (_mag_vector.y - mag_offset.y);
			_mag_field.z	  = (_mag_vector.z - mag_offset.z);
			
			//_backend->filter_raw_sample_gyro(FXas21002, _gyro_vector, _dt_gyro);
			filter_raw_sample_accel(0, _accel_vector, _dt_gyro);
			filter_raw_sample(_mag_vector, _dt_gyro);

			uint64_t _last_time = micros();
			_dt_gyro		    = static_cast<float>(_last_time - _last_update_us) * 1e-6f;			
			_last_update_us     = _last_time;
	
			//(_accel_vector * 0.488f * 1e-3f * 9.81).print();
			//(_mag_field * 0.1f).print();		
		}
	}
}


void AP_FXOS8700::update()
{
	// signify backend to update front end with filtered data
	if(_have_sens == false)	return;
	
	update_accel (0);
	update_magnetometer();
}
