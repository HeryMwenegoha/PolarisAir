#include "AP_Arduimu.h"
#include "mavlink.h"
#include "Vector3.h"

void AP_Arduimu::initialise()//HardwareSerial *_Port) 
{
	//Port = _Port;
	Serial.println(F("ArduImu Setup"));
	Serial2.begin(115200);
	delay(50);
}

void AP_Arduimu::update()
{
	_ins_backend->update_gyro (MPU6000);
	_ins_backend->update_accel(MPU6000);	
	_compass_backend->update_magnetometer();
}

void AP_Arduimu::accumulate()
{
	#if MAV_APPROACH
	mavlink_message_t msg;
	mavlink_status_t status;
	
	while(Serial2.available())
	{
		uint8_t read_bytes = Serial2.read();  
		if(mavlink_parse_char(MAVLINK_COMM_2, read_bytes, &msg, &status))
		{
			switch(msg.msgid)
			{         
				case MAVLINK_MSG_ID_RAW_IMU:
				
				vector3f _mag_field;
				vector3f _gyro_vector;
				vector3f _accel_vector;
				
				_accel_vector.x  =  static_cast<float>(mavlink_msg_raw_imu_get_xacc(&msg))  *  1.0f; // LSB
				_accel_vector.y  =  static_cast<float>(mavlink_msg_raw_imu_get_yacc(&msg))  * -1.0f;
				_accel_vector.z  =  static_cast<float>(mavlink_msg_raw_imu_get_zacc(&msg))  * -1.0f;
				_gyro_vector.x 	 =  static_cast<float>(mavlink_msg_raw_imu_get_xgyro(&msg)) *  1.0f;
				_gyro_vector.y   =  static_cast<float>(mavlink_msg_raw_imu_get_ygyro(&msg)) * -1.0f;
				_gyro_vector.z   =  static_cast<float>(mavlink_msg_raw_imu_get_zgyro(&msg)) * -1.0f;
				_mag_field.x     =  static_cast<float>(mavlink_msg_raw_imu_get_xmag(&msg))	* -1.0f;
				_mag_field.y     =  static_cast<float>(mavlink_msg_raw_imu_get_ymag(&msg))  *  1.0f;
				_mag_field.z     =  static_cast<float>(mavlink_msg_raw_imu_get_zmag(&msg))  * -1.0f;  
				
				// Serial.println(time -_last_update_msec);
				_last_update_msec = millis();				
				_ins_backend->filter_raw_sample_gyro(MPU6000 , _gyro_vector , 0.02);
				_ins_backend->filter_raw_sample_accel(MPU6000, _accel_vector, 0.02);
				_compass_backend->filter_raw_sample(_mag_field, 0.02);
				
				
				//_accel_vector.printV();
				// _gyro_vector.printV();
				// _mag_field.printV();						
				break;
			}
		}
	}
	
	#else
	
	while(Serial2.available()){
		
		uint8_t _read   = Serial2.read();
		if(serialhelper(_read)){
			confirmed_pax = false;
		    index 		  = 0;
			
			/*	
			for(int i = 0; i < 6; i++){
				Serial.print(buffer[i]); Serial.print("  ");
			}	
			Serial.println(" ");
			*/
			
			if(buffer[5] == 27){						
				decode _decode;
								
				// decoded buffer
				for(int i = 1; i < 26; i++){
					_decode.buffer[i] = buffer[6 + i];
				}				
				// CRC calculation excludes startsign as well as CRC0 and CRC1
				// The CRC length = n + 5 i.e. n - payload length, 5 - header length excluding startsign
				// CRC extra use MESSAGE_CRC_EXTRA[message_id] defined in common to get the extra byte calculation of CRC
				uint16_t crc  = crc_calculate(&buffer[1], 31);
				crc_accumulate(144, &crc);
				
				uint8_t  CKL = (uint8_t)(crc & 0xFF);
				uint8_t  CKH = (uint8_t)(crc >> 8);		
					
				if((CKL == buffer[32]) && (CKH == buffer[33])){
					vector3f _mag_field;
					vector3f _gyro_vector;
					vector3f _accel_vector;							

					_accel_vector.x  =  static_cast<float>(_decode.message.xacc)  *  1.0f; // LSB
					_accel_vector.y  =  static_cast<float>(_decode.message.yacc)  * -1.0f;
					_accel_vector.z  =  static_cast<float>(_decode.message.zacc)  * -1.0f;
					_gyro_vector.x 	 =  static_cast<float>(_decode.message.xgyro) *  1.0f;
					_gyro_vector.y   =  static_cast<float>(_decode.message.ygyro) * -1.0f;
					_gyro_vector.z   =  static_cast<float>(_decode.message.zgyro) * -1.0f;
					_mag_field.x     =  static_cast<float>(_decode.message.xmag)  * -1.0f;
					_mag_field.y     =  static_cast<float>(_decode.message.ymag)  *  1.0f;
					_mag_field.z     =  static_cast<float>(_decode.message.zmag)  * -1.0f; 		
					
					
		
					_last_update_msec = millis();
					_ins_backend->filter_raw_sample_gyro(MPU6000 , _gyro_vector , 0.02);
					_ins_backend->filter_raw_sample_accel(MPU6000, _accel_vector, 0.02);
					_compass_backend->filter_raw_sample(_mag_field, 0.02);		

				//_accel_vector.printV();
				//_gyro_vector.printV();
				// _mag_field.printV();						
				}						
			}
			break;
		}
	}
	
	#endif
	
	/*@ Heartbeat Packets 
	 *@ Send Heartbeats every second
	 */
	every_second();
}


bool AP_Arduimu::serialhelper(uint8_t _read){
	
	switch(_read){
		case 0xFE:
			if(index < 34){
				if((confirmed_pax == false) && (index == 0)){
					buffer[index++] = _read;
				}else{
					buffer[index++] = _read;
				}
			}
			return (index == 34);
		
		case 26:
			if(index < 34){
				if(index == 1){
					confirmed_pax   = true;
					buffer[index++] = _read;
				}			
				else if(index > 1){
					buffer[index++] = _read;	
				}
			}
			return (index == 34);
		
		default:
			if((index < 34) && (confirmed_pax == true) && (index > 1)){
				buffer[index++] = _read;	
			}
			return (index == 34);		
	}	
}

void AP_Arduimu::every_second()
{
	uint32_t _time = millis();
	if(_time - _update_msec >= 700){

		 _update_msec = _time; 
		 byte buffer[3];
		 buffer[0] = 0x65;
		 buffer[1] = 0x66;
		 buffer[2] = 0x67;
		 Serial2.write(buffer,3);	
	}
}
