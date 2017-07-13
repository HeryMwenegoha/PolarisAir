#pragma once
#include "AP_Parameters.h"
#include "Vectors.h"
#include "Common.h"

class AP_INS_Backend;

class AP_INS
{
	friend class AP_INS_Backend;
	
	public:
	AP_INS():
	_hil_mode(false),
	_have_ins(false),
	_calibrated(false),
	_calibrate_run(0),
	_calibrate_count(0),
	_num_sensors(0),
	_parameters(&AP_params)
	{
		for(int i =0; i<3; i++)
		{
			_gyro[i].zero(); 
			_accel[i].zero(); 
			_raw_gyro[i].zero(); 
			_raw_accel[i].zero();
			_accel_scale.zero();
			_gyro_offset[i].zero();
			_accel_offset[i].zero();
			_offset_initial[i].zero();
			_accel_health_count[i] = false;
			_gyro_health_count[i] = false; 
			_product_id[i] = 0;
			_accel_[i].zero();
			_gyro_[i].zero();
			
			_filtered_gyro[i].zero();
			_filtered_accel[i].zero();
			_new_gyro_data[i] = false;
			_new_accel_data[i] = false;
		}
		#if VERSION0bbb
		_low_pass_cutoff_gyro  = 50;  // Used to be 50Hz
		_low_pass_cutoff_accel = 10;  // Used to be 10Hz	
		#else
		// I have used this with no problems
		_low_pass_cutoff_gyro  = 20;  // Used to be 50Hz - Ardupilot uses 20Hz for gyro filter
		_low_pass_cutoff_accel = 7;   // Used to be 10Hz - Ardupilot Uses 10Hz for accel filter
		#endif
		instance 			   = 0;
	}
	
	void	 update();	
	vector3f* gyro();
	vector3f* accel();
	vector3f* raw_gyro();
	vector3f* raw_accel();
	
	
	void 	 setHil(const float &_rr, const float &_pr, const float &_yr, const float &_xacc, const float &_yacc, const float &_zacc);
	bool  	 have_ins();
	uint32_t last_update_msec();
	bool     healthy();
	
	void     register_device(byte product_id, uint16_t _speed)
	{
		_product_id[_num_sensors++] = product_id;
	}
	
	vector3f _gyro_[3];  // ADC -> back-end data
	vector3f _accel_[3]; // ADC -> back-end data
	bool    _gyro_health_count[3];
	bool    _accel_health_count[3];
	
	protected:
	AP_Storage *_parameters;
	
	bool  	 calibrate();
	bool     _calibrated;
	byte     _calibrate_count;
	byte     _calibrate_run;
	vector3f _gyro[3];		// rad/s 
	vector3f _accel[3];	    // m/s/s 
	vector3f _raw_gyro[3];	// ADC
	vector3f _raw_accel[3]; // ADC
	byte 	 _num_sensors;
	bool     _have_ins;
	bool     _hil_mode;
	
	float _low_pass_cutoff_gyro;
	float _low_pass_cutoff_accel;	
	vector3f _filtered_gyro[3];
	vector3f _filtered_accel[3];	
	bool _new_gyro_data[3];
	bool _new_accel_data[3];
	
	private:
	uint8_t  _product_id[3];
	vector3f _accel_scale;
	//vector3f _gyro_scale[3];	
	vector3f _gyro_offset[3];
	vector3f _accel_offset[3];
	vector3f _offset_initial[3];
	uint32_t _last_update_msec;
	
	AP_INS_Backend *_backends[3];
	uint8_t instance;
	
	void add_backend(AP_INS_Backend *);
};

extern  AP_INS      AP_ins;