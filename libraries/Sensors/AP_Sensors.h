#pragma once
#include "L3GD20.h"
#include "LSM303D.h"
#include "LSM303A.h"
//#include "MPU9250.h"
//#include "AP_Arduimu.h"
//#include "AK8963.h"
//#include "L3GD20H.h"

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
		//_mpu9250.initialise();
		//_ak8963.initialise();
		//_l3gd20h.initialise();
		#else
		//_mpu6000.initialise();//&Serial2);
		#endif
	}
	
	void accumulate()
	{
		#if ARDU == 0
		_l3gd20.accumulate();
		_lsm303A.accumulate();
		_lsm303D.accumulate();	
		//_mpu9250.accumulate();
		//_ak8963.accumulate();
		//_l3gd20h.accumulate();
		#else
		//_mpu6000.accumulate();	
		#endif
	}
	
	void update()
	{
		#if ARDU == 0
		_l3gd20.update();
		_lsm303A.update();
		_lsm303D.update();	
		//_mpu9250.update();
		//_ak8963.update();
		//_l3gd20h.update();		
		#else
		//_mpu6000.update();
		#endif
	}
	
	private:
	AP_L3GD20  _l3gd20;
	AP_LSM303D _lsm303D;
	AP_LSM303A _lsm303A;
	//AP_Arduimu _mpu6000;
	//AP_MPU9250	_mpu9250;
	//AP_AK8963	_ak8963;
	//AP_L3GD20H  _l3gd20h;
};
