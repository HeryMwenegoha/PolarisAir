#pragma once
#include "AP_INS.h"
#include "AP_INS_Backend.h"
#include "Wire.h"
#include "Vectors.h"
/*@ This is the MPU9250 Gyro upraged from MPU6500 with Noise Rate Density of 0.011dps/sqrt(Hz) Address :  b1101000 with ADO low or  b1101001 with ADO high
 *@ 16 Bit-rate output
 *@ 8 Bit temperature output
 *@ DLPF && DHPF
 *@ 250|500|1000|2000
 *@ 65.5 LSB/dps for 500dps FS
 *@ +/-15 dps Zero Rate Level at 500dps 
 *@ ODR(Output Data Rate) 4 - 8000 Hz
 *@ OTR(Operating Temperature Range) -40 | +85
 *@ TODR 1Hz
 *@ SUC(Supply Current) 6.1mA 
 *@ I2C OR 100KHz | 400KHz
 */
 
/*@ This is the MPU9250 Accelerometer upraged from MPU6500 with Noise Rate Density of 0.011dps/sqrt(Hz) Address :  b1101000 with ADO low or  b1101001 with ADO high
 *@ 16 Bit-rate output
 *@ 8 Bit temperature output
 *@ DLPF && DHPF
 *@ 2|4|8|16
 *@ 8192 LSB/g for 4G's
 *@ 70mg Zero Rate Level 
 *@ ODR(Output Data Rate) 4 - 4000 Hz
 *@ OTR(Operating Temperature Range) -40 | +85
 *@ TODR 1Hz
 *@ SUC(Supply Current) 6.1mA 
 *@ I2C OR 100KHz | 400KHz
 */
 
 // Follow NED Convention
 
class AP_MPU9250 : public  AP_INS_Backend 
{
	public:
	AP_MPU9250():
	_backend(&AP_ins_backend)
	{
		_last_update_us = 0;
		_dt   			= 0;
		_update_us 		= 0;
		_have_sens      = false;
	}	
	bool initialise();
	void accumulate();
	void update();
		
	private:
	AP_INS_Backend *_backend; 	
	uint64_t _update_us;
	float	 _dt;
	uint64_t _last_update_us;	
	bool 	 _have_sens;
};