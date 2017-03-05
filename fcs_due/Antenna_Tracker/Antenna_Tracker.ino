#include <mavlink.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "Common.h"

Adafruit_MotorShield AFMS        = Adafruit_MotorShield(); 
Adafruit_StepperMotor *tiltMotor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *panMotor  = AFMS.getStepper(200, 1);


uint32_t _last_msec              = 0;
uint32_t _last_slow_loop_msec    = 0;
float groundspeed                = 0;

float   bearing                  = 0;
float   cumulative_bearing       = 0;
uint8_t _initial_bearing_flag    = 0;
uint8_t sync                     = 0;
uint8_t fix_type                 = 0;
Location nextWP, homeWP;

float _initial_bearing          = 0;
float _last_bearing              = 0;
float _last_tilt                 = 0;
float tilt_rad                   = 0;

void setup(){
  Serial.begin(38400);   
  AFMS.begin();
  
  tiltMotor->setSpeed(100);
  panMotor->setSpeed(100);

  _last_msec           = millis();
  _last_slow_loop_msec = millis();
}


void loop()
{
  /*
}
  String msg = "";

  while(Serial.available()){
    char c = Serial.read();

    msg += c;
    delay(1);
  }

  if(msg != ""){
    int val = msg.toInt();
    Serial.println(val);
    if(val >= 0){
      tiltMotor->step(val, FORWARD, DOUBLE);
    }
    else{
      tiltMotor->step(-val, BACKWARD, DOUBLE);
    }
  }
  */
  
  mavlink_process();
  slow_loop();
}

void tiltstep_double(float _tilt_steps){
  int8_t _t_step    = static_cast<int8_t>(_tilt_steps);
  if(_t_step >= 0)
     tiltMotor->step(_t_step, FORWARD ,  DOUBLE);   
  else
     tiltMotor->step((-_t_step), BACKWARD , DOUBLE);   
}

void panstep_double(float _pan_steps){
  int8_t _p_step         = static_cast<int8_t>(_pan_steps);
  if(_p_step >= 0)
   panMotor->step(_p_step, FORWARD ,  DOUBLE);   
  else
   panMotor->step((-_p_step), BACKWARD , DOUBLE); 
}

void tiltstep_micro(float _tilt_steps){
  int8_t _t_step    = static_cast<int8_t>(_tilt_steps);
  if(_t_step >= 0)
     tiltMotor->step(_t_step, FORWARD ,  MICROSTEP);   
  else
     tiltMotor->step((-_t_step), BACKWARD , MICROSTEP);   
}

void panstep_micro(float _pan_steps){
  int8_t _p_step         = static_cast<int8_t>(_pan_steps);
  if(_p_step >= 0)
   panMotor->step(_p_step, FORWARD ,  MICROSTEP);   
  else
   panMotor->step((-_p_step), BACKWARD , MICROSTEP); 
}



void slow_loop(){
  uint32_t now = millis();
  if((now - _last_slow_loop_msec) >= 2000){
    _last_slow_loop_msec = now;

  // Pan Angle
  if(_NE_distance( homeWP,  nextWP).length() >= 5.0f){
    if(sync > 0     &&  fix_type > 1 && _initial_bearing_flag < 4){
       _initial_bearing = ToDeg(_NE_bearing(homeWP,  nextWP));
       _initial_bearing =  wrap_180(_initial_bearing);
       _last_bearing    = _initial_bearing;
       _initial_bearing_flag++;
    }
  }

  // First Obtain the Bearing
  if(_initial_bearing_flag < 4){
  return;
  }

  // Pan 
  float _current_bearing = ToDeg(_NE_bearing(homeWP,  nextWP));
  _current_bearing       = wrap_180(_current_bearing);
  float _delta           = _current_bearing - _last_bearing; 
  float _pan_steps       = _delta * 200.0f/360.0f;
  panstep_double(_pan_steps);

  
  // Tilt
  float alt_difference   = fabs(nextWP.alt - homeWP.alt);
  float distance         = _NE_distance(homeWP,  nextWP).length();
  if(distance != 0)
  {
    float _tilt       = ToDeg(atan2(alt_difference, distance));
    _tilt             = wrap_180(_tilt);
    float _delta_tilt = _tilt - _last_tilt;
    float _tilt_steps = _delta_tilt * 200.0f/360.0f;
    tiltstep_double(_tilt_steps);
   }
  }
}


void mavlink_process(){
  uint32_t now = millis();
  if((now - _last_msec) >= 20)
  {
    _last_msec = now;
    mavlink_status_t status;
    mavlink_message_t msg;
    uint8_t read_bytes = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, read_bytes, &msg, &status))
    {
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          }
          break;
  
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
          float lat    = static_cast<float>(mavlink_msg_gps_raw_int_get_lat(&msg) *1e-7f);
          float lon    = static_cast<float>(mavlink_msg_gps_raw_int_get_lon(&msg) *1e-7f);
          uint16_t alt = static_cast<uint16_t>(mavlink_msg_gps_raw_int_get_alt(&msg) *1e-3f);
          groundspeed  = static_cast<float>(mavlink_msg_gps_raw_int_get_vel(&msg) *1e-2f);
          fix_type     = mavlink_msg_gps_raw_int_get_fix_type(&msg);
  
          if(fix_type > 1){
            nextWP.lat = lat;
            nextWP.lon = lat;
            nextWP.alt = alt;
          }
  
          if(fix_type > 1 && sync < 10){
            sync ++;
          }
  
          if(sync > 2     && sync < 4){
            homeWP.lat   = lat;
            homeWP.lon   = lon;
            homeWP.alt   = alt;
          }
          }
          break;

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:{
               float pan_   = mavlink_msg_local_position_ned_get_x(&msg);
               float tilt_  = mavlink_msg_local_position_ned_get_y(&msg);
               panstep_double(pan_);
               tiltstep_double(tilt_);
           }
           break;
         
      }
    }
  }
}

