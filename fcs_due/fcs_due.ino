//#include <mavlink.h>
#define  CHARS_TABLE 1
#include <AP_Parameters.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <AP_Compass.h>
#include <AP_GPS.h>  
#include <AP_AHRS.h>
#include <AP_PID.h>
#include <AP_Radio.h>
#include <AP_Sensors.h>
#include <Servo.h>
#include <AP_BatteryVolts.h>
#include <AP_HgtFilter_AQ.h>
#include "AP_Define.h"
#include "AP_WayPoint.h"
#include "AP_L1_Controller.h"
#include "AP_TECS.h"   // Change hgtcompfilter
#include "AP_Program.h"
#include "AP_Mavlink.h"
#include <avr/pgmspace.h>


#if PLANE_TYPE == 100
#error Change the Refresh Rate and Number of Servos in "Servo.h" , otherwise you will burn your servos.
#endif

AP_Radio    AP_radio;
AP_Airspeed AP_airspeed(&(AP_params.ParameterStorage.list.arspdEnabled));
AP_Baro     AP_baro;
AP_GPS      AP_gps;
AP_Sensors  AP_sensors; // carries IMU and Compass
AP_AHRS     AP_ahrs(&AP_airspeed, AP_gps, &AP_ins, &AP_compass, AP_baro); // GPS is passed by reference
AP_WayPoint AP_waypoint;
//AP_L1       AP_l1(AP_ahrs);
//AP_TECS     AP_tecs(AP_ahrs);
AP_Program  AP_program(&AP_params, &AP_ahrs, &AP_gps, &AP_waypoint, /*&AP_tecs, &AP_l1, */&AP_radio);
AP_BatteryVolts AP_batteryvolts;
AP_Mavlink  AP_mavlink(AP_params , AP_ahrs , AP_gps,AP_ins,AP_compass,AP_airspeed, AP_baro, AP_waypoint, AP_program, AP_batteryvolts);


uint64_t last_stream_usec;
bool     led_state;
uint32_t blink_msec;
byte     medium_loop_counter;
#define  LEDPIN 12

void setup()
{
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  
  Serial.println(F("BOOT"));

  AP_params.initialise(&Serial);  
  AP_waypoint.initialise();
  AP_mavlink.initialise(&Serial);
  AP_gps.initialise(&Serial1);
  AP_sensors.initialise(); 
  AP_baro.initialise();
  AP_program.initialise();

  mainloop_lastTime  = millis() - mainloop_rate;
  last_stream_usec   = micros();
}

void loop()
{ 
  precise_timing_loop();
  uint32_t mainloop_currentTime = millis();
  if(mainloop_currentTime - mainloop_lastTime >= mainloop_rate){
    float Gdt         = mainloop_currentTime - mainloop_lastTime;
    mainloop_lastTime = mainloop_currentTime;    
    if(Gdt > 50){
      Serial.print(F("Loop Speed Error: ")); 
      Serial.println(Gdt);
    }
    fast_loop();
    medium_loop();
    blink_led();
  }
}


void precise_timing_loop()
{
  #if DSM_REMOTE
  uint64_t now = micros();
  float DT = (now - last_stream_usec) * 1e-3f;
  if(DT   >= 5){
    last_stream_usec  = now;
    AP_dsm.read_stream();
  }
  #endif
   
  // Read incoming GPS stream
  AP_gps.read();

  // Accumulate sensor readings
  AP_sensors.accumulate();
  
  // Read incoming stream from Serial0
  AP_mavlink.readstream();
}

void fast_loop()
{
  // 50Hz Sensors
  // Read BMP180
  AP_baro.read();
  
  // Get backend object and update front end objects
  AP_sensors.update();

  // Process Front end compass values from raw mag values into proper mag fields
  AP_compass.update();

  // Process Front end imu raw values into proper scaled imu values (rotation rate and accelerations)
  AP_ins.update();

  // Perform GPS based direct cosine matrix calculations with scaled compass and imu readings to get euler angles
  AP_ahrs.update();

  // Autopilot Programme
  AP_program.update(_servo_out);

  // Send mavlink stream
  AP_mavlink.sendstream();
}

void medium_loop(){
  switch(medium_loop_counter)
  {
    case 0:
      medium_loop_counter++;
      AP_batteryvolts.read();
      break;

    case 1:
      medium_loop_counter++;
      break;

    case 2:
      medium_loop_counter++;
      break;

    case 3:
      medium_loop_counter++;
      break;

    case 4:
      medium_loop_counter = 0;
      AP_baro.update();
      break;
  }
}

void blink_led()
{
  led_counter++;
  byte events = 0;
  float battery_voltage = AP_batteryvolts.get_adc() * AP_params.ParameterStorage.list.PowerModule_Gain; 
  if(battery_voltage < 10.75)
  {
    events = 1;
  }
  else
  {
    events = 0;
  }
  
  switch(events)
  {
    case 0:
    blinker(1000);
    break;

    case 1:
    blinker(100);
    break;
  }

}

void blinker (uint16_t time_delay){
  time_delay = time_delay/2;
  uint32_t now = millis();
  if((now - blink_msec) >= time_delay){
     blink_msec  = now;
     led_state   = !led_state;
     led_state == true ? analogWrite(LEDPIN, 220) : analogWrite(LEDPIN, 0);
  }  
}

void print_gps_time(){
  #if PRINT 
  PRINT(AP_gps.hours());
  PRINT("   ");
  PRINT(AP_gps.minutes());
  PRINT("   ");
  PRINTLN(AP_gps.seconds());
  #endif
}

void print_tecs(){
  #if PRINT // only prints if tecs update_pitch_throttle is running.
  PRINT(AP_ahrs.altitude_estimate());
  PRINT("   ");
  PRINTLN(AP_program.filtered.altitude);
  #endif
}


void print_radio(){
  PRINT(AP_radio.chan1());
  PRINT(F("   "));
  PRINT(AP_radio.chan2());
  PRINT(F("   "));
  PRINT(AP_radio.chan3());
  PRINT(F("   "));
  PRINT(AP_radio.chan4());   
  PRINT(F("   "));
  PRINTLN(AP_radio.chan8()); 
}

void print_euler(){
  PRINT(ToDeg(AP_ahrs.roll));
  PRINT("  ");
  PRINT(ToDeg(AP_ahrs.pitch));
  PRINT("  ");
  PRINT(ToDeg(AP_ahrs.yaw));
  PRINT("  ");
  PRINT(ToDeg(AP_ahrs.rollrate));
  PRINT("  ");
  PRINT(ToDeg(AP_ahrs.pitchrate));
  PRINT("  "); 
  PRINTLN(ToDeg(AP_ahrs.yawrate)); 
}
