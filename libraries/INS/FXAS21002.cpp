/* (C) Hery A Mwenegoha UoN 2019
*/

#include "FXAS21002.h"
#define ADRESS		B00100001 //0x43
#define PRODUCT_ID  215
#define WHO_AM_I	0x0C


/* CTRL_REG0	0x0D
 * Item		Bit 	Bit-Value	Effect-Value
 * BW   	[7:6]	00			32Hz @ 100Hz ODR
 * SPIW		[5]		0			4-Wire mode
 * SEL		[4:3]	00			HPF - N.ADRESS
 * HPF_EN	[2]		0			HPF - Disabled
 * FS 		[1:0]	10			15.625mdps/LSB @ +/-500dps
 */
#define CTRL_REG0				 0x0D
#define CTRL_REG0_SETUP			 B00000010


/* CTRL_REG1	0x13
 * Item		Bit 	Bit-Value	Effect-Value
 * -	   	[7]		0			-
 * RST		[6]		0			No Device Reset
 * ST		[5]		0			Self-Test Disabled
 * DR		[4:2]	011			100Hz ODR
 * ACTIVE 	[1]		0			Standby Mode
 * READY 	[0]		0			Not Reading Registers
 */
#define CTRL_REG1				 0x13
#define DR 						 2
#define ACTIVE                   1
#define CTRL_REG1_SETUP			 (0x03 << DR) | (0x00 << ACTIVE)//B00001100
#define CTRL_REG1_ACTIVE	     (0x03 << DR) | (0x01 << ACTIVE)//B00001110
#define CTRL_REG1_STANDBY	      0x00							//B00000000

/* CTRL_REG2	0x14  
 * Item				Bit 	Bit-Value	Effect-Value
 * INT_CFG_FIFO	   	[7]		0			To INT2
 * INT_EN_FIFO		[6]		0			FIFO INT DIS
 * INT_CFG_RT		[5]		0			TO INT2
 * INT_EN _RT		[4]		0			RT INT DIS
 * INT_CFG_DRDY 	[3]		0			To INT2 
 * INT_EN_DRDY 		[2]		0			DR INT DIS
 * IPO_L			[1]		0			ACTIVE LOW
 * PP_O_D			[0]		0			PUSH-PULL
 */
#define CTRL_REG2				 0x14
#define CTRL_REG2_SETUP			 B00000000 // Default


/* CTRL_REG3	0x15
 * Item				Bit 	Bit-Value	Effect-Value
 * -	  		 	[7]		0			-
 * -				[6]		0			-
 * -				[5]		0			-
 * -				[4]		0			-
 * WRAPTOONE 		[3]		0			ROLL OVER TO 0x00
 * EXTCTRLEN 		[2]		0			External POwer Mode - INT2 output
 * -				[1]		0			ACTIVE LOW
 * FS_DOUBLE		[0]		0			No FS dobling
 */
#define CTRL_REG3				 0x15
#define WRAPTOONE 				 3
#define EXTCTRLEN 				 2
#define CTRL_REG3_SETUP			 (0x00 << WRAPTOONE) | (0x00 << EXTCTRLEN) | 0x00 // Default


/* TEMP 0x12
 * Range -128 +128
 */
#define OUT_TEMP	0x12

#define STATUS_REG	0x00  // or 0x07
#define OUT_X_H		0x01
#define OUT_X_L		0x02
#define OUT_Y_H		0x03
#define OUT_Y_L		0x04
#define OUT_Z_H		0x05
#define OUT_Z_L		0x06


AP_FXAS21002 *AP_FXAS21002::instance = NULL;


bool AP_FXAS21002::initialise()
{
	//Serial.println(instance!=NULL);
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
	
	
	// Reset Device
	Wire.write8(ADRESS, CTRL_REG1, (1<<6));
	delay(15);	
	Wire.write8(ADRESS, CTRL_REG1, (1<<6));
	delay(15);
	
	
	// FS selection
	Wire.write8(ADRESS, CTRL_REG0, (byte)CTRL_REG0_SETUP);		
	delay(15);	
	Wire.write8(ADRESS, CTRL_REG0, (byte)CTRL_REG0_SETUP);		
	delay(15);
		
	
	// ODR and activate
	Wire.write8(ADRESS, CTRL_REG1, (byte)CTRL_REG1_ACTIVE);		
	delay(100);	
	Wire.write8(ADRESS, CTRL_REG1, (byte)CTRL_REG1_ACTIVE);		
	delay(100);
		
	//_imu->register_device(PRODUCT_ID, 760);
	
	_last_update_us = micros();
	_update_us      = micros();
	Serial.println(F("AP_FXAS21002::	Gyro	Ready"));
	_have_sens = true;
	return true;	
}


void AP_FXAS21002::accumulate()
{
	// gather data and update backend with new filtered data info
	if(!_have_sens)	return;
	
 	uint64_t _time = micros();
	if(_time - _update_us >= 10000)
	{
		_update_us = _time;
		uint8_t buffer[6];	
		
		Wire.beginTransmission(ADRESS);
		Wire.write((STATUS_REG | 0x80));
		Wire.endTransmission();
		
		Wire.requestFrom(ADRESS, 7);
	    uint32_t _start_msec = millis();
		while(Wire.available() < 7){
			if((millis() - _start_msec) >= 50){
				Serial.println(F("FXAS21002-G	I2C	Timeout"));
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
			
			vector3f _gyro_vector;
			
			_gyro_vector.x =  (int16_t)(buffer[0] << 8 | buffer[1]);
			_gyro_vector.y = -(int16_t)(buffer[2] << 8 | buffer[3]);
			_gyro_vector.z = -(int16_t)(buffer[4] << 8 | buffer[5]);
				
			if(_dt_gyro > 1.0f){
				_dt_gyro = 0.01f;
			}
			
			//_backend->filter_raw_sample_gyro(FXas21002, _gyro_vector, _dt_gyro);
			filter_raw_sample_gyro(0, _gyro_vector, _dt_gyro);

						
			uint64_t _last_time = micros();
			_dt_gyro		    = static_cast<float>(_last_time - _last_update_us) * 1e-6f;			
			_last_update_us     = _last_time;
			
			/*
			Serial.print(_gyro_vector.x * 15.625f * 1e-3f); Serial.print(F("\t"));
			Serial.print(_gyro_vector.y * 15.625f * 1e-3f); Serial.print(F("\t"));
			Serial.println(_gyro_vector.z * 15.625f * 1e-3f); 
			*/
			//_gyro_vector.print();
			int8_t Temp = Wire.read8(ADRESS, OUT_TEMP, false); // return uint8_t [0-127 Celsius]
			
			/*
			Serial.print(Temp); 
			Serial.print(F("\t"));
			(_gyro_vector * 15.625f * 1e-3f).print();
			*/
		}
	}
}


void AP_FXAS21002::update()
{
	// signify backend to update front end with filtered data
	if(_have_sens == false)	return;
	
	update_gyro (0);
}
