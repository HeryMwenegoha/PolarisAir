#pragma once
#include "AP_INS.h"
#include "AP_INS_Backend.h"
#include "Wire.h"
#include "Vectors.h"
/*@ This is the L3GD20H upraged from L3GD20 with Noise Rate Density of 0.011dps/sqrt(Hz) Gyro Address Dec: 107, Hex: 0x6B
 *@ 16 Bit-rate output
 *@ 8 Bit temperature output
 *@ DLPF && DHPF
 *@ 250|500|2000
 *@ 17.50 mdps/digit for 500dps
 *@ +/-15 dps Zero Rate Level at 500dps 
 *@ ODR(Output Data Rate) 95|190|380|760 Hz
 *@ OTR(Operating Temperature Range) -40 | +85
 *@ TODR 1Hz
 *@ SUC(Supply Current) 6.1mA 
 *@ I2C OR 100KHz | 400KHz
 */
class AP_L3GD20 : public  AP_INS_Backend 
{
	public:
	AP_L3GD20():
	_backend(&AP_ins_backend)
	{
		_last_update_us = 0;
		_dt_gyro   		= 0;
		_update_us 		= 0;
		_have_sens      = false;
	}	
	bool initialise();
	void accumulate();
	void update();
		
	private:
	AP_INS_Backend *_backend; 	
	uint64_t _update_us;
	float	 _dt_gyro;
	uint64_t _last_update_us;
	
	bool _have_sens;
};