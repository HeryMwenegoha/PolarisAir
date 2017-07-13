#include "LSM303A.h"
#define ACCEL_ADDRESS 				0x19

#define CTRL_REG1_A	  				0x20
#define REG1A_100Hz_Normal_XYZEN    B01010111 

#define CTRL_REG4_A	  				0x23
#define REG4A_4G_HR   				B10011000

#define CTRL_REG5_A					0x24
#define FIFO_EN					   (0x00 << 6)

#define FIFO_CTRL_REG_A				0x2E
#define FIFO_STREAM_MODE		   (0x02 << 6)

#define FIFO_SRC_REG_A				0x2F
#define FIFO_FULL					0x40
#define FIFO_EMPTY					0x20
#define FIFO_MASK					0x1f

#define STATUS_REG_A  				0x27
#define OUT_X_L_A	  				0x28


bool AP_LSM303A::initialise()
{
	/*
     * Set ODR to 100HZ
	 */
	_i2c->write8(ACCEL_ADDRESS, CTRL_REG1_A, (byte)(REG1A_100Hz_Normal_XYZEN));
	delay(15);
	_i2c->write8(ACCEL_ADDRESS, CTRL_REG1_A, (byte)(REG1A_100Hz_Normal_XYZEN));
	delay(15);		
	if(_i2c->read8(ACCEL_ADDRESS, CTRL_REG1_A) != 0x57){
		//Serial.println(F("AP_LSM303A:	Accelerometer	Not	Found"));
		_have_sens = false;
		return false;
	}delay(10);

	
	/*
	 * Set FSR to +/-4G
	 */
	_i2c->write8(ACCEL_ADDRESS, CTRL_REG4_A, (byte)(REG4A_4G_HR));
	delay(10);
	_i2c->write8(ACCEL_ADDRESS, CTRL_REG4_A, (byte)(REG4A_4G_HR));
	delay(10);
		
	_last_accel_update_us = micros();
	_update_us = micros();
	
	Serial.println(F("AP_LSM303A::	Accel	Ready"));
	_have_sens = true;
	return true;	
}

void AP_LSM303A::update()
{	
	if(!_have_sens) return;
	_backend->update_accel(L3Gd20);
}

void AP_LSM303A::accumulate()
{
	if(!_have_sens) return;
	
	// 100Hz Accelerometer
	uint64_t _time = micros();
	if(_time - _update_us >= 10000)
	{
		_update_us = _time;
		
		#if NEW_IMPLEMENTATION
		const uint8_t _status  = _i2c->read8(ACCEL_ADDRESS, FIFO_SRC_REG_A);
		uint8_t num_of_samples = 0;
		
		if(_status & FIFO_FULL){
			num_of_samples = 32;
			//Serial.println("Full");
		}
		else if(_status & FIFO_EMPTY){
			num_of_samples = 0;
			//Serial.println("Empty");
		}
		else {
			num_of_samples = _status & FIFO_MASK;
			//Serial.println(num_of_samples);
		}
		
		if(num_of_samples >= 32){
			int16_t  sec_buffer[num_of_samples][3];
			vector3f _accel_vector;
			struct bytes2_u{
				byte bytes[2];			
				int16_t int16() {
					return (int16_t)(this->bytes[1] << 8 | this->bytes[0]) >> 4;
				}
			};	
			
			bytes2_u temp;
			
			_i2c->beginTransmission(ACCEL_ADDRESS);
			_i2c->write((OUT_X_L_A | 0x80));
			_i2c->endTransmission();
			
			_i2c->requestFrom(ACCEL_ADDRESS, 32);
		    if(_i2c->available() >= num_of_samples ){	
				for(int i = 0; i < 5; i++){				
					temp.bytes[0]    = _i2c->read();
					temp.bytes[1]    = _i2c->read();				
					sec_buffer[i][0] = temp.int16(); 
					
					temp.bytes[0]    = _i2c->read();
					temp.bytes[1]    = _i2c->read();				
					sec_buffer[i][1] = -temp.int16(); 
					
					temp.bytes[0]    = _i2c->read();
					temp.bytes[1]    = _i2c->read();				
					sec_buffer[i][2] = -temp.int16(); 
									
					_accel_vector.x  +=  sec_buffer[i][0];
					_accel_vector.y  +=  sec_buffer[i][1];
					_accel_vector.z  +=  sec_buffer[i][2];
				}
			}
			
			_accel_vector = _accel_vector/5;
			
			_accel_vector = _accel_vector * 2 * 1e-3f * 8192.0f; // Convert to MPU6000 scaling factor 8192 LSB/G
			
			//_accel_vector.print();

			_backend->filter_raw_sample_accel(L3Gd20, _accel_vector, _dt_accel);		
						
			uint64_t _last_time    = micros();
			_dt_accel 			   = static_cast<float>(_last_time - _last_accel_update_us) * 1e-6f;
			_last_accel_update_us  = _last_time;
		}
		
		#else
			
		const uint8_t _status = ((_i2c->read8(ACCEL_ADDRESS, STATUS_REG_A) & B00001000) >> 3);
		if(_status == 1)
		{			
			uint8_t buffer[6];
			
			_i2c->beginTransmission(ACCEL_ADDRESS);
			_i2c->write((OUT_X_L_A | (1 << 7)));
			_i2c->endTransmission();
			
			_i2c->requestFrom(ACCEL_ADDRESS, 6);
			uint32_t _start_msec = millis();
			while(_i2c->available() < 6){
				
				if((millis() - _start_msec) >= 50){
					Serial.println("LSM303A	I2C	Timeout");
					return;
				}
				
			}
			
			buffer[0] = _i2c->read();
			buffer[1] = _i2c->read();
			buffer[2] = _i2c->read();
			buffer[3] = _i2c->read();
			buffer[4] = _i2c->read();
			buffer[5] = _i2c->read();			
		
			// no need to drop bits since its 12bit precision 
			vector3f _accel_vector;
			_accel_vector.x =  (int16_t)(buffer[1] << 8 | buffer[0]);
			_accel_vector.y = -(int16_t)(buffer[3] << 8 | buffer[2]);
			_accel_vector.z = -(int16_t)(buffer[5] << 8 | buffer[4]);		

			//_accel_vector.x = (_accel_vector.x /8192) * 9.81f;
			//_accel_vector.y = (_accel_vector.y /8192) * 9.81f;
			//_accel_vector.z = (_accel_vector.z /8192) * 9.81f;

			//_accel_vector.print();			
			
			if(_dt_accel  > 1.0f)
				_dt_accel = 0.01f;
			
			_backend->filter_raw_sample_accel(L3Gd20, _accel_vector, _dt_accel);											
			
			uint64_t _last_time    = micros();
			_dt_accel 			   = static_cast<float>(_last_time - _last_accel_update_us) * 1e-6f;
			_last_accel_update_us  = _last_time;					
		}
		#endif
	}
}
