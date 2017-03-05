#pragma once
#include "L3GD20.h"
#include "LSM303D.h"
#include "LSM303A.h"
#include "AP_Arduimu.h"

#define ARDU	0
class AP_Sensors
{	
	public:
	AP_Sensors()
	{
		
	}
	
	void initialise()
	{
		#if ARDU == 0
		_l3gd20.initialise();
		_lsm303A.initialise();
		_lsm303D.initialise();
		#else
		_mpu6000.initialise();//&Serial2);
		#endif
	}
	
	void accumulate()
	{
		#if ARDU == 0
		_l3gd20.accumulate();
		_lsm303A.accumulate();
		_lsm303D.accumulate();	
		#else
		_mpu6000.accumulate();	
		#endif
	}
	
	void update()
	{
		#if ARDU == 0
		_l3gd20.update();
		_lsm303A.update();
		_lsm303D.update();	
		#else
		_mpu6000.update();
		#endif
	}
	
	private:
	AP_L3GD20  _l3gd20;
	AP_LSM303D _lsm303D;
	AP_LSM303A _lsm303A;
	AP_Arduimu _mpu6000;
};
