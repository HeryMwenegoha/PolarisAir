#include "L3GD20.h"
#define ADRESS		0x6B
#define PRODUCT_ID  107
#define WHO_AM_I	0x0F

/* CTRL_REG1    0x20
 * Item  Bit	Value
 * ODR  (7:6)  	95Hz 
 * BW   (5:4)  	25Hz 
 * PD   (3)	  	Normal  
 * Zen  (2)	  	Enabled
 * Yen  (1)	  	Enabled
 * Xen  (0)	  	Enabled
 * ======================================================
 * DR 	BW 	ODR(Hz)	BW(HZ)
 * 00 	00 	95 		12.5
 * 00 	01 	95 		25
 * 00 	10 	95 		25
 * 00 	11 	95 		25
 * 01 	00 	190 	12.5
 * 01 	01 	190 	25
 * 01 	10 	190 	50
 * 01 	11 	190 	70
 * 10 	00 	380 	20
 * 10 	01 	380 	25
 * 10 	10 	380 	50
 * 10 	11 	380 	100
 * 11 	00 	760 	30
 * 11 	01 	760 	35
 * 11 	10 	760 	50
 * 11 	11 	760 	100
 *========================================================
 * DR1-DR0 Output data rate selection. Refer to Table 21 (above)
 * BW1-BW0 Bandwidth selection.        Refer to Table 21 (above)
 * PD Power-down mode enable. Default value: 0 (0: power-down mode, 1: normal mode or sleep mode)
 * Zen Z axis enable. Default value: 1 (0: Z axis disabled; 1: Z axis enabled)
 * Yen Y axis enable. Default value: 1 (0: Y axis disabled; 1: Y axis enabled)
 * Xen X axis enable. Default value: 1 (0: X axis disabled; 1: X axis enabled)
 */
#define CTRL_REG1				 0x20
#define CTRL_REG1_SETUP			 B00111111



/* CTRL_REG2  	0x21
 * Item  Bit	Value
 * MASK (7:6)  	Set to zero
 * HPM  (5:4)  	Normal
 * HPCF (3:0)	0.09Hz  
 * ================================================
 * HPM(5:4) High-pass filter mode
 * 0 0 		Normal mode (reset reading HP_RESET_FILTER)
 * 0 1 	 	Reference signal for filtering
 * 1 0 	 	Normal mode
 * 1 1 	 	Autoreset on interrupt event 

 * ===================================================
 * HPCF(3:0) ODR=95Hz ODR=190Hz ODR=380Hz 	ODR=760 Hz
 * 0000 	 7.2 	  13.5 		27 			51.4
 * 0001 	 3.5 	  7.2 		13.5 		27
 * 0010 	 1.8 	  3.5 		7.2 		13.5
 * 0011 	 0.9 	  1.8 		3.5 		7.2
 * 0100 	 0.45 	  0.9 		1.8 		3.5
 * 0101 	 0.18 	  0.45 		0.9 		1.8
 * 0110 	 0.09 	  0.18 		0.45 		0.9
 * 0111 	 0.045 	  0.09 		0.18 		0.45
 * 1000 	 0.018 	  0.045 	0.09 		0.18
 * 1001 	 0.009 	  0.018 	0.045 		0.09
 */
#define CTRL_REG2	 		0x21
#define CTRL_REG2_SETUP	 	B00001001



/* CTRL_REG3  0x22
 * Item  	  Bit	Value
 * l1_Int1   (7)   	Disable
 * l1_Boot   (6)  	Disable
 * H_Lactive (5)	Disable  
 * PP_OD   	 (4) 	Disable
 * I2_DRDY   (3)  	Disable
 * I2_WTM 	 (2)	Disable  
 * I2_ORun   (1)  	Disable
 * I2_Empty  (0)  	Disable
 * ================================================
 * I1_Int1 		Interrupt enable on INT1 pin. 			Default value: 0 (0: disable; 	1: enable)
 * I1_Boot 		Boot status available on INT1. 			Default value: 0 (0: disable; 	1: enable)
 * H_Lactive 	Interrupt active configuration on INT1. Default value: 0 (0: high;	 	1:low)
 * PP_OD 		Push-pull / Open drain. 				Default value: 0 (0: push- pull;1: open drain)
 * I2_DRDY 		Date-ready on DRDY/INT2. 				Default value: 0 (0: disable; 	1: enable)
 * I2_WTM 		FIFO watermark interrupt on DRDY/INT2. 	Default value: 0 (0: disable; 	1: enable)
 * I2_ORun 		FIFO overrun interrupt on DRDY/INT2 	Default value: 0 (0: disable; 	1: enable)
 * I2_Empty 	FIFO empty interrupt on DRDY/INT2. 		Default value: 0 (0: disable; 	1: enable)
 */
#define CTRL_REG3			0x22
#define CTRL_REG3_SETUP		B00000000
	
	

/* CTRL_REG4  	0x23
 * Item  Bit	Value
 * BDU   (7) 	1
 * BLE   (6)  	0
 * FS 	 (5:4)	01  
 * RES   (3)    -
 * MASK  (2:1)  00
 * SIM 	 (0)	0 
 * ================================================
 * BDU 	   Block data update. 				 	Default value: 0	(0: continuos update; 1: output registers not updated until MSb and LSb reading)
 * BLE 	   Big/little endian data selection. 	Default value: 0.	(0: Data LSb @ lower address; 1: Data MSb @ lower address)
 * FS1-FS0 Full scale selection. 				Default value: 00	(00: 250 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps)
 * SIM 	   SPI serial interface mode selection. Default value: 0	(0: 4-wire interface; 1: 3-wire interface).
 */	
#define CTRL_REG4			0x23
#define FS	4
#define BDU 7
#define CTRL_REG4_SETUP	    (0x01 << FS) | (0x01 << BDU) 



/* CTRL_REG5  	0x24
 * Item  	Bit		Value
 * BOOT   	(7) 	0
 * FIFO_EN  (6)  	0
 * MASK		(5)		-
 * HPen 	(4)		0  
 * INT1_Sel (3:2)   -
 * Out_Sel  (1:0)   00
 * ================================================
 * BOOT 				Reboot memory content. 			Default value: 0(0: normal mode;	 1: reboot memory content)
 * FIFO_EN 				FIFO enable. 					Default value: 0(0: FIFO disable; 	 1: FIFO Enable)
 * HPen 				High-pass filter enable. 		Default value: 0(0: HPF disabled; 	 1: HPF enabled See Figure 20)
 * INT1_Sel1-INT1_Sel0	INT1 selection configuration. 	Default value: 0(See Figure 20)
 * Out_Sel1-Out_Sel0 	Out selection configuration. 	Default value: 0(See Figure 20)
 */
#define CTRL_REG5		 0x24
#define FIFO	6
#define HPen 	4
#define CTRL_REG5_SETUP	(0x00 << FIFO) | (0x00 << HPen)

#define REFERENCE	0x25
#define OUT_TEMP	0x26
#define STATUS_REG	0x27
#define OUT_X_L		0x28
#define OUT_X_H		0x29
#define OUT_Y_L		0x2A
#define OUT_Y_H		0x2B
#define OUT_Z_L		0x2C
#define OUT_Z_H		0x2D

bool AP_L3GD20::initialise()
{
	// check product ID
	if(PRODUCT_ID != (Wire.read8(ADRESS, WHO_AM_I) >> 1)){
		//Serial.println(F("AP_L3GD20:	Gyro	Not	Found"));
		_have_sens = false;
		return false;
	}
	delay(10);
	
	Wire.write8(ADRESS, CTRL_REG1, (byte)CTRL_REG1_SETUP);		
	delay(15);	
	Wire.write8(ADRESS, CTRL_REG1, (byte)CTRL_REG1_SETUP);		
	delay(15);
		
	Wire.write8(ADRESS, CTRL_REG2, (byte)CTRL_REG2_SETUP);		
	delay(5);	
	Wire.write8(ADRESS, CTRL_REG2, (byte)CTRL_REG2_SETUP);		
	delay(5);
	
	Wire.write8(ADRESS, CTRL_REG3, (byte)CTRL_REG3_SETUP);		
	delay(5);	
	Wire.write8(ADRESS, CTRL_REG3, (byte)CTRL_REG3_SETUP);		
	delay(5);
	
	Wire.write8(ADRESS, CTRL_REG4, (byte)CTRL_REG4_SETUP);		
	delay(5);	
	Wire.write8(ADRESS, CTRL_REG4, (byte)CTRL_REG4_SETUP);		
	delay(5);
	
	Wire.write8(ADRESS, CTRL_REG5, (byte)CTRL_REG5_SETUP);		
	delay(5);	
	Wire.write8(ADRESS, CTRL_REG5, (byte)CTRL_REG5_SETUP);		
	delay(5);	
	//_imu->register_device(PRODUCT_ID, 760);
	
	_last_update_us = micros();
	_update_us      = micros();
	Serial.println(F("AP_L3GD20::	Gyro	Ready"));
	_have_sens = true;
	return true;	
}

void AP_L3GD20::update()
{
	if(_have_sens == false)	return;
	
	_backend->update_gyro (L3Gd20);
}

void AP_L3GD20::accumulate()
{
	if(!_have_sens)	return;
	
 	uint64_t _time = micros();
	if(_time - _update_us >= 10000)
	{
		_update_us = _time;
		const uint8_t _status = ((Wire.read8(ADRESS, STATUS_REG) & 0x08) >> 3);
		if(_status == 1)
		{					
			uint8_t buffer[6];	
			Wire.beginTransmission(ADRESS);
			Wire.write((OUT_X_L | 0x80));
			Wire.endTransmission();
			
			Wire.requestFrom(ADRESS, 6);
			uint32_t _start_msec = millis();
			while(Wire.available() < 6){
				if((millis() - _start_msec) >= 50){
					Serial.println("L3GD20-G	I2C	Timeout");
					return;
				}
			}
			
			buffer[0] = Wire.read();
			buffer[1] = Wire.read();				
			buffer[2] = Wire.read();
			buffer[3] = Wire.read();	
			buffer[4] = Wire.read();
			buffer[5] = Wire.read();	
			
			vector3f _gyro_vector;
			
			_gyro_vector.x =  (int16_t)(buffer[1] << 8 | buffer[0]);
			_gyro_vector.y = -(int16_t)(buffer[3] << 8 | buffer[2]);
			_gyro_vector.z = -(int16_t)(buffer[5] << 8 | buffer[4]);
				
			if(_dt_gyro > 1.0f){
				_dt_gyro = 0.01f;
			}		
			_backend->filter_raw_sample_gyro(0, _gyro_vector, _dt_gyro);
						
			uint64_t _last_time = micros();
			_dt_gyro		    = static_cast<float>(_last_time - _last_update_us) * 1e-6f;			
			_last_update_us     = _last_time;
			
			//_gyro_vector.print();
			//(_gyro_vector * 17.5f * 1e-3f).print();
						
		}
	}
}
