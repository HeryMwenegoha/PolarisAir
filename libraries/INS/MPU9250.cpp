#include "MPU9250.h"
#define ADRESS				B1101000 // ADO Low - 0x68
#define PRODUCT_ID  		0x73
#define WHO_AM_I			0x75
#define SMPLRT_REG			0x19
#define SMPLRT_SET			9
#define CONFIG_REG			0x1A
#define CONFIG_SET			2
#define GYRO_CONFIG_REG		0x1B
#define GYRO_CONFIG_SET		((1 << 3) | (0 << 1) | 0)
#define ACCEL_CONFIG_REG	0x1C
#define ACCEL_CONFIG_SET	(1 << 3)
#define USER_CTRL_REG		0x6A
#define USER_CTRL_SET		((B100 << 4) | 000) // Disable MST_I2C Mode
#define PWR_MGMT_1_REG		0x6B
#define PWR_MGMT_1_SET		(1<<7) | 2
#define PWR_MGMT_2_REG		0x6C
#define PWR_MGMT_2_SET		((0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) |(0 << 1) | 0)
#define INT_PIN_CFG_REG		0x37
#define INT_PIN_CFG_SET		(1 << 1)
#define I2C_MST_CNTL_REG	0x24
#define INT_PIN_CFG_SET		(1 << 1)
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48

bool AP_MPU9250::initialise()
{
	// PWR1
	Wire.write8(ADRESS, PWR_MGMT_1_REG, (byte)PWR_MGMT_1_SET);		
	delay(15);	
	Wire.write8(ADRESS, PWR_MGMT_1_REG, (byte)PWR_MGMT_1_SET);		
	delay(15);

	// PWR2
	Wire.write8(ADRESS, PWR_MGMT_2_REG, (byte)PWR_MGMT_2_SET);		
	delay(15);	
	Wire.write8(ADRESS, PWR_MGMT_2_REG, (byte)PWR_MGMT_2_SET);			
	delay(15);
	
	// USER CNTRL
	Wire.write8(ADRESS, USER_CTRL_REG, (byte)USER_CTRL_SET);		
	delay(15);	
	Wire.write8(ADRESS, USER_CTRL_REG, (byte)USER_CTRL_SET);		
	delay(15);
		
	// BYPASS MODE ENABLED
	Wire.write8(ADRESS, INT_PIN_CFG_REG, INT_PIN_CFG_SET);
	delay(15);
	Wire.write8(ADRESS, INT_PIN_CFG_REG, INT_PIN_CFG_SET);
	delay(15);
	
	// check product ID
	if(PRODUCT_ID != Wire.read8(ADRESS, WHO_AM_I)){
		_have_sens = false;
		return false;
	}
	delay(5);	
	
	// SMPLRT RATE
	Wire.write8(ADRESS, SMPLRT_REG, (byte)SMPLRT_SET);		
	delay(5);	
	Wire.write8(ADRESS, SMPLRT_REG, (byte)SMPLRT_SET);			
	delay(5);
	
	// CONFIG BITS
	Wire.write8(ADRESS, CONFIG_REG, (byte)CONFIG_SET);		
	delay(5);	
	Wire.write8(ADRESS, CONFIG_REG, (byte)CONFIG_SET);		
	delay(5);

	// FS and DLPF
	Wire.write8(ADRESS, GYRO_CONFIG_REG, (byte)GYRO_CONFIG_SET);		
	delay(5);	
	Wire.write8(ADRESS, GYRO_CONFIG_REG, (byte)GYRO_CONFIG_SET);			
	delay(5);
	
	// Accel Config
	Wire.write8(ADRESS, ACCEL_CONFIG_REG, (byte)ACCEL_CONFIG_SET);		
	delay(5);	
	Wire.write8(ADRESS, ACCEL_CONFIG_REG, (byte)ACCEL_CONFIG_SET);		
	delay(5);	
	
	_last_update_us = micros();
	_update_us      = micros();
	Serial.println(F("AP_MPU9250::	IMU	Ready"));
	_have_sens = true;
	return true;	
}

void AP_MPU9250::update()
{
	if(_have_sens == false)	return;
	
	_backend->update_gyro (MPU6000);
	_backend->update_accel(MPU6000);
}

void AP_MPU9250::accumulate()
{
	if(!_have_sens)	return;
	
 	uint64_t _time = micros();
	if(_time - _update_us >= 10000)
	{
		_update_us = _time;
		
		uint8_t buffer[6];
		vector3f _accel_vector;	
		vector3f _gyro_vector;
		
		// ACCELEROMETER	
		{		
		Wire.beginTransmission(ADRESS);
		Wire.write((ACCEL_XOUT_H | 0x80));
		Wire.endTransmission();
		
		Wire.requestFrom(ADRESS, 6);
		uint32_t _start_msec = millis();
		while(Wire.available() < 6){
			if((millis() - _start_msec) >= 50){
				Serial.println("MPU9250-A	I2C	Timeout");
				return;
			}
		}
		buffer[0] = Wire.read();
		buffer[1] = Wire.read();				
		buffer[2] = Wire.read();
		buffer[3] = Wire.read();	
		buffer[4] = Wire.read();
		buffer[5] = Wire.read();	
			
		_accel_vector.x =  (int16_t)(buffer[2] << 8 | buffer[3]);
		_accel_vector.y =  (int16_t)(buffer[0] << 8 | buffer[1]);				
		_accel_vector.z = -(int16_t)(buffer[4] << 8 | buffer[3]);	
		}
		
		
		// GYROSCOPES		
		{
		Wire.beginTransmission(ADRESS);
		Wire.write((GYRO_XOUT_H | 0x80));
		Wire.endTransmission();
		
		Wire.requestFrom(ADRESS, 6);
		uint32_t _start_msec = millis();
		while(Wire.available() < 6){
			if((millis() - _start_msec) >= 50){
				Serial.println("MPU9250-G	I2C	Timeout");
				return;
			}
		}
		buffer[0] = Wire.read();
		buffer[1] = Wire.read();				
		buffer[2] = Wire.read();
		buffer[3] = Wire.read();	
		buffer[4] = Wire.read();
		buffer[5] = Wire.read();	
		
		_gyro_vector.x =  (int16_t)(buffer[2] << 8 | buffer[3]);
		_gyro_vector.y =  (int16_t)(buffer[0] << 8 | buffer[1]);				
		_gyro_vector.z = -(int16_t)(buffer[4] << 8 | buffer[3]);	
		}		
		
		// Filter
		if(_dt > 1.0f){
			_dt = 0.01f;
		}
		_backend->filter_raw_sample_gyro(MPU6000,  _gyro_vector,   _dt);
		_backend->filter_raw_sample_accel(MPU6000, _accel_vector,  _dt);			
		uint64_t _last_time = micros();
		_dt		    = static_cast<float>(_last_time - _last_update_us) * 1e-6f;			
		_last_update_us     = _last_time;
		//_gyro_vector        = _gyro_vector * (1/65.5);
		//_gyro_vector.print();	
		//_accel_vector.print();	
	}
}
