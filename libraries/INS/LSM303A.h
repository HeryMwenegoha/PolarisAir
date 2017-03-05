#pragma once
#include "AP_INS.h"
#include "AP_INS_Backend.h"
#include "Wire.h"
#include "Vectors.h"

/*@ LSM303 Accelerometer Dec:25, Hex : 0x19
 *@ +/- 2g|4g|8g|16g Selectable FS 
 *@ 16 Bit Output 12Bit Precision (High Resolution), Lowest 4 Bits are 0.
 *@ 100KHz - 400KHz I2C communication
 *@ OTR -40C to +85C
 *@ Zero G Level +/- 60mg at +/-2G's FS
 *@ Sensitivity: 2mg/LSB   at +/-4G's FS
 *@ SUC: 110uA @ 50Hz ODR
 *@	In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddress
	field. In other words, SUB(7) must be equal to 1 while SUB(6-0) represents the
	address of the first register to be read.
 *@	Note:
 *@ Accel - LSM303 ODR REG1A_1344Hz_Normal_XYZEN
 *@ Accel - LSM303 GRAVITY REG4A_4G_HR @ 2mg/LSB
 *@ Gyro  - L3GD20 ODR (Output Data Rate) 760Hz @ BW 100Hz normal mode with XYZ enaled 
 *@ Gyro  - L3GD20 FSR (Full Scale Range) range to +/- 500 degrees per second
 */
class AP_LSM303A : public  AP_INS_Backend 
{
	public:
	AP_LSM303A():
	_backend(&AP_ins_backend),
	_i2c(&Wire)
	{
		_dt_accel 	    	 = 0;
		_last_accel_update_us=0;
		_update_us			 = 0;
		_have_sens = false;
	}
	
	bool initialise();
	void accumulate();
	void update();
		
	private:
	AP_INS_Backend *_backend; 
	TwoWire *_i2c;	
	float	 _dt_accel;
	uint64_t _update_us;
	uint64_t _last_accel_update_us;
	
	bool _have_sens;

};
