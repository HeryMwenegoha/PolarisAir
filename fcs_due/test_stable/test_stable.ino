#include <AP_Parameters.h>
#include <AP_Airspeed.h>
#include <mavlink.h>
#include <AP_Sensors.h>
#include <AP_GPS.h> 
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_DSM.h>
#include <Servo.h>
#include <DueTimer.h>
#define HEADER_LEN      8
#define UAV             1

AP_DSM      AP_dsm;
AP_Sensors  AP_sensors;
AP_Airspeed AP_airspeed(&(AP_params.ParameterStorage.list.arspdEnabled));
AP_GPS      AP_gps;
AP_Baro     AP_baro;
AP_AHRS     AP_ahrs(&AP_airspeed, AP_gps, &AP_ins, &AP_compass, AP_baro); // GPS is passed by reference

 Servo servo_roll;
 Servo servo_pitch;
 Servo servo_throttle;
 Servo servo_yaw;
 Servo servo_gcs1; // gcs1 - controlled by gcs
 Servo servo_gcs2; // gcs2 - controlled by gcs

uint64_t _update_usec;
uint32_t loop_lapse_time;
byte     _stream_speed_10Hz = 0;
byte    _stream_speed_3Hz  = 0;
byte     _stream_speed_1Hz  = 0;
static  boolean SEND_MAV_DATA = false;

typedef short      i16;
typedef unsigned short u16;
typedef long i32;
typedef unsigned long u32;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  AP_params.initialise();
  AP_gps.initialise(&Serial3);  
  AP_dsm.initialise(&Serial1);
  AP_sensors.initialise();
  AP_baro.initialise();
  _update_usec = micros();
}

uint64_t last_stream_usec = 0;
void loop() {
  // put your main code here, to run repeatedly:
  AP_gps.read();
  AP_sensors.accumulate();
  AP_baro.read();
  
  uint64_t now = micros();

  float DT = (now - last_stream_usec) * 1e-3f;
  if(DT   >= 5){
    last_stream_usec  = now;
    AP_dsm.read_stream();
  }
  
  now = micros();
  if((now - _update_usec) >= 20000){
    if((now - _update_usec) > 50000){
      Serial.println(F("Loop Took Long"));
    }
    
    _update_usec = now;
    AP_sensors.update();
    AP_compass.update();
    AP_ins.update();
    AP_ahrs.update();

    blink_led();
    sendstream();
  }
  
  readstream(&Serial);
}

uint32_t led_counter = 0;
void blink_led(){
  led_counter++;
  if(led_counter == 25 && digitalRead(13) == HIGH){
    digitalWrite(13, LOW);
  }else if(led_counter == 50 && digitalRead(13) == LOW){
    led_counter = 0;
    digitalWrite(13, HIGH);
  }
}


void readstream(HardwareSerial *Port)
{
 mavlink_message_t msg;
 mavlink_status_t status;
 while(Port->available())
 {
  uint8_t read_bytes = Port->read();  
  if(mavlink_parse_char(MAVLINK_COMM_0, read_bytes, &msg, &status))
  {
    switch(msg.msgid)
    {      
       case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
       SEND_MAV_DATA = true;
       AP_params.SendParamList(Port, UAV);
       break;
       
       case MAVLINK_MSG_ID_PARAM_SET:
       if(mavlink_msg_param_set_get_target_system(&msg) == UAV)
       {
         char param_id[16];
         float param_value = mavlink_msg_param_set_get_param_value(&msg);
         mavlink_msg_param_set_get_param_id(&msg, param_id);
         AP_params.UpdateStorage(Port, UAV, param_id, param_value);
       }
       break;            
    }
  }
 }
}



 void send_hb(boolean &Send_Allowed)
 {   
     mavlink_message_t  msg_t;
     uint8_t  buf[MAVLINK_MSG_ID_HEARTBEAT_LEN + HEADER_LEN];     // Pax 9 + 8 = 17bytes
     
     mavlink_msg_heartbeat_pack(
     UAV, MAV_COMP_ID_ALL, &msg_t,
     MAV_TYPE_FIXED_WING, 
     MAV_AUTOPILOT_GENERIC,
     AP_ahrs.get_flightmode(),    // base mode
     MAV_MODE_PREFLIGHT,          // custom mode
     MAV_STATE_STANDBY);
     
     uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
     
     if(Send_Allowed){
         Serial.write(buf,len);
     }
 }

 void send_euler(uint32_t &loop_lapse, boolean &Send_Allowed)
 {   
     mavlink_message_t  msg_t;
     uint8_t  buf[MAVLINK_MSG_ID_ATTITUDE_LEN + HEADER_LEN];     // PAX 28 + 8 = 36bytes
     
     mavlink_msg_attitude_pack(
     UAV, MAV_COMP_ID_IMU, &msg_t,
     loop_lapse, 
     AP_ahrs.roll, 
     AP_ahrs.pitch, 
     AP_ahrs.yaw,  // true heading as calculated by dcm
     AP_ahrs.rollrate, 
     AP_ahrs.pitchrate, 
     AP_ahrs.yawrate);
   
     uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
     
     if(Send_Allowed){
         Serial.write(buf,len);
     }
 }


 void send_imuraw(uint32_t &loop_lapse, boolean &Send_Allowed)
 {   
     mavlink_message_t  msg_t;
     uint8_t  buf[MAVLINK_MSG_ID_RAW_IMU_LEN + HEADER_LEN];     // PAX 26 + 8 = 34 bytes 
     
     mavlink_msg_raw_imu_pack(
     UAV, MAV_COMP_ID_IMU, &msg_t,
     loop_lapse,
     static_cast<int16_t>(AP_ins.raw_accel()[1].x), 
     static_cast<int16_t>(AP_ins.raw_accel()[1].y),
     static_cast<int16_t>(AP_ins.raw_accel()[1].z), 
     static_cast<int16_t>(AP_ins.raw_gyro()[1].x), 
     static_cast<int16_t>(AP_ins.raw_gyro()[1].y), 
     static_cast<int16_t>(AP_ins.raw_gyro()[1].z), 
     static_cast<int16_t>(AP_compass.raw_field().x), 
     static_cast<int16_t>(AP_compass.raw_field().y), 
     static_cast<int16_t>(AP_compass.raw_field().z)); 
     
     uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
     
     if(Send_Allowed){
         Serial.write(buf,len);
     }
 }

 void send_gps(uint32_t &loop_lapse,  boolean &Send_Allowed)
 {
     mavlink_message_t  msg_t;
     uint8_t  buf[MAVLINK_MSG_ID_GPS_RAW_INT_LEN + HEADER_LEN];     //  PAX 30 + 8 = 38 bytes
     mavlink_msg_gps_raw_int_pack(
     UAV, MAV_COMP_ID_GPS, &msg_t,
     loop_lapse, 
     AP_gps.status(), 
     static_cast<i32>(AP_gps.location().lat*1e7), 
     static_cast<i32>(AP_gps.location().lon*1e7),
     static_cast<i32>(AP_gps.altitude()    *1e3), 
     static_cast<u16>(AP_gps.horizontal_dilution()),
     static_cast<u16>(AP_gps.vertical_dilution()),
     static_cast<u16>(AP_gps.groundspeed()  *1e2),
     static_cast<u16>(AP_gps.heading()      *1e2), // COG
     AP_gps.num_sats());
     
     uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t); 
     
     if(Send_Allowed){
         Serial.write(buf,len);
     }
 }
 
 
 void sendstream()
 {
   stream_10Hz();
 }
 
 void stream_10Hz()
 {
   switch(_stream_speed_10Hz)
   {
     case 0:
     _stream_speed_10Hz++;
     send_euler(loop_lapse_time, SEND_MAV_DATA);
     break;
     
     case 1:
     _stream_speed_10Hz++;
     break;
     
     case 2:
     _stream_speed_10Hz++;
     break;
     
     case 3:
     _stream_speed_10Hz++;
     #if HIL_SIM == 1
     //send_servo(loop_lapse_time, &_servo_out, SEND_MAV_DATA);
     #endif
     break;
     
     case 4:
     _stream_speed_10Hz++;
     break;
     
     case 5:
     _stream_speed_10Hz++;
     send_imuraw(loop_lapse_time, SEND_MAV_DATA);
     break;
        
     case 6:
     stream_3Hz();
     _stream_speed_10Hz = 0;
     break;
   }   
 }

// 2Hz
 void stream_3Hz()
 {
   switch(_stream_speed_3Hz)
   {
     case 0:
     _stream_speed_3Hz++;
     break;
     
     case 1:
     _stream_speed_3Hz++;
     #if HIL_SIM == 0
     //send_servo(loop_lapse_time, &_servo_out, SEND_MAV_DATA);
     #endif
     break;
     
     case 2:
      _stream_speed_3Hz++;
      //send_vfr(loop_lapse_time, &_servo_out, SEND_MAV_DATA);
     break;
     
     case 3:
     stream_1Hz();
     _stream_speed_3Hz = 0;
     break;
   }   
 }


 void stream_1Hz()
 {
   switch(_stream_speed_1Hz)
   {
     case 0:
     _stream_speed_1Hz++;
     send_hb(SEND_MAV_DATA);
     break;
     
     case 1:
     _stream_speed_1Hz++;
     send_gps(loop_lapse_time,  SEND_MAV_DATA);
     break;
     
     case 2:
     _stream_speed_1Hz = 0;
     break;
   }   
 }

