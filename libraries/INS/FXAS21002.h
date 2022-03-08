#pragma once
#include "AP_INS.h"
#include "AP_INS_Backend2.h"
#include "Wire.h"
#include "Vectors.h"
/*@ This is the FXAS21002 upraged from NXP with Noise Rate Density of 0.025dps/sqrt(Hz) Gyro Address Dec: 107, Hex: 0x20
 *@ 16 Bit-rate output
 *@ 8 Bit temperature output

 *@ DLPF && DHPF
 *@ 250|500|2000
 *@ 15.625 mdps/digit for 500dps
 *@ +/-25 dps Zero Rate Level at 500dps 
 *@ ODR(Output Data Rate) 95|190|380|760 Hz
 *@ OTR(Operating Temperature Range) -40 | +85
 *@ TODR 1Hz
 *@ SUC(Supply Current) 6.1mA 
 *@ I2C OR 100KHz | 400KHz
 */
 
 // MOUNTED AS PER DATASHEET
class AP_FXAS21002 : public  AP_INS_Backend2 
{
	public:
	AP_FXAS21002()
	{
		_last_update_us = 0;
		_dt_gyro   		= 0;
		_update_us 		= 0;
		_have_sens      = false;
	    
		SCALER          = (15.625f * 1e-3f); // In Backend Class - LSB2DPS
		TYPE            = FXAS21002;		 //  
		CHAR            = 'G';				 // Gyro
	}	
	bool initialise();
	void accumulate();
	void update();
	
	static AP_FXAS21002* get_instance(){
		if(!instance)
			instance = new AP_FXAS21002;
		return instance;	
	}
		
	private:	
	static 			AP_FXAS21002* instance;
	uint64_t 		_update_us;
	float	 		_dt_gyro;
	uint64_t 		_last_update_us;
	bool 			_have_sens;
};