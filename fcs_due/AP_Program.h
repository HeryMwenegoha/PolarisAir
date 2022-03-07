#pragma once
//#include "AP_Mavlink.h"

/*
 * Notes:
 * Change Pins to Follow the Mega 2560 Convention
 */
#define  rollpin      30
#define  pitchpin     32
#define  throttlepin  34
#define  yawpin       36

#define  gcs1pin      38 
#define  gcs2pin      40

#define  aux1pin      42
#define  aux2pin      44

class AP_Program
{
   public:
   AP_Program(AP_Storage *,  AP_AHRS *, AP_GPS *, AP_WayPoint *,/* AP_TECS *, AP_L1 *,*/ AP_Radio *);
   void initialise();
   void update(servo_out &ServoChan);
   
   AP_TECS &tecs(){
    return AP_tecs;
   }
   
   AP_L1   &l1(){
    return  AP_l1;
   }
   
   AP_AHRS &ahrs(){
    return *_ahrs;
   }

   const AP_HgtFilter_AQ &aq(){
    return AP_hgtfilter_aq;
   }

   const Location &Set_WP(){
    return WPB;
   }

   Location &CurrentWP(){
    return WPCurrent;
   }

   void Mission_Start(){
     guided(false);  // automatically sets mission complete to false
     seq = 0;
     is_autolanding = false;
     is_autotakeoff = false;
     gps_takeoff_enabled  = false;
   }

   // do isnan check, isinf check as well - rogue initialisation
   void CMD_ModeChange(uint16_t _mode){
    gcs_mode_change = _mode;
   }
   

   void guided(const bool &val){
     do_guided       = val;
     guided_complete = false;
     do_guided == false ? mission_complete = false : mission_complete = true;
   }

   void guided(const bool &val, const Location &locCurrent, const Location &locNext, const uint16_t _loiter_seconds){
     loiter_seconds = _loiter_seconds;
     WPCurrent = locCurrent;
     WPB       = locNext;
     do_guided = val;
     guided_complete = false;
     do_guided == false ? mission_complete = false : mission_complete = true;
   }
   
   uint16_t loiter_seconds;
   byte flight_mode;
   struct _filtered{
   float altitude;
   float climbrate;
   };
   _filtered filtered;
   byte _flagship;
     
   private:
   AP_Storage  *_parameter; 
   AP_AHRS     *_ahrs;
   AP_GPS      *_gps;
   AP_WayPoint *_waypoint; 
   //AP_TECS     _tecs;
   //AP_L1       _l1;
   AP_Radio        *_radio;
   AP_L1           AP_l1;//(AP_ahrs);
   AP_TECS         AP_tecs;//(AP_ahrs);
   AP_HgtFilter_AQ AP_hgtfilter_aq;
   
   bool         mission_complete;
   byte         seq;
   int         _last_seq;
   uint64_t     update_stab_usec;
   uint64_t     auto_usec;
   uint64_t     guided_usec;
   boolean      home_loiter;
   boolean      do_guided;
   boolean      guided_complete;
   boolean      is_autolanding;
   boolean      is_autotakeoff;
   boolean      _hdg_hold;
   Location     WPB;
   Location     WPCurrent;
   Location     WPA; // only for automatic control
   bool         gps_takeoff_enabled;

   uint16_t     gcs_mode_change; // Consider removing this 
  
   float throttle;  
   float yaw_dem; 
   AP_RollController  AP_rollcontroller;
   AP_PitchController AP_pitchcontroller;
   AP_SteerController AP_steercontroller;
   AP_YawController   AP_yawcontroller;

   Servo servo_roll;
   Servo servo_pitch;
   Servo servo_throttle;
   Servo servo_yaw;
   Servo servo_gcs1; // gcs1 - controlled by gcs
   Servo servo_gcs2; // gcs2 - controlled by gcs

   float roll_stabilise(float, float );
   float pitch_stabilise(float, float );
   float yaw_stabilise(float, float, float );
   float throttle_stabilise(float);
   float _spd_scaler();

   float roll_navigation(float, float );
   float pitch_navigation(float, float );
   float yaw_navigation(float,  float);
   float throttle_navigation(float);
   
   float roll_nav_stickmix(float);
   float pitch_nav_stickmix(float); 

   Location one_km_waypoint(Location prevWP, float bearing);
   bool distance_to_completion_check(const Location &WPB);
   bool cross_the_finish_line_check(const Location &WPA, const Location &WPB);
   
   void  manual_control(servo_out &ServoChan);
   bool  degrees_tohold(float &bearing);
   //float ground_steer(servo_out &ServoChan);
   void  stabilised_control(servo_out &ServoChan);
   void  automatic_control(servo_out &ServoChan);
   void  loiter_waypoint(servo_out &ServoChan, Location WPLoiter, float altitude);
   void  guided_control(servo_out &ServoChan);
   void  guided_navigation(servo_out &ServoChan);
};


AP_Program::AP_Program(AP_Storage *ap_params, AP_AHRS *ap_dcm, AP_GPS * ap_gps, AP_WayPoint *ap_wp, /*AP_TECS *ap_tecs, AP_L1 *ap_l1,*/ AP_Radio *ap_radio):
AP_l1           (*ap_dcm),
AP_tecs         (*ap_dcm),
AP_hgtfilter_aq (*ap_dcm, ap_params),
AP_yawcontroller(*ap_dcm)  
{
  AP_rollcontroller.initialise(ap_params,  ap_dcm);
  AP_pitchcontroller.initialise(ap_params, ap_dcm);
  AP_steercontroller.initialise(ap_params, ap_gps);
  
  flight_mode = PREFLIGHT; 
  _parameter  = ap_params;
  _ahrs       = ap_dcm;
  _gps        = ap_gps;
  _waypoint   = ap_wp;
  //_tecs       = ap_tecs;
  //_l1         = ap_l1;
  _radio      = ap_radio;
  _hdg_hold   = false;
   mission_complete     = false;

   _flagship = 0;
}


void
AP_Program::initialise(){
  seq            =  0;
  _last_seq      =  -1;
  home_loiter    = false;
  do_guided      = false;
  guided_complete= false;
  is_autolanding = false;
  is_autotakeoff = false;
  gps_takeoff_enabled  = false;
  auto_usec      = micros();
  guided_usec    = micros();

  update_stab_usec = micros();
  throttle         = 1000;

  gcs_mode_change = 900; // IGNORE MODE INITIALISED
  
  servo_roll.attach(rollpin,1000,2000);
  servo_pitch.attach(pitchpin,1000,2000);
  servo_throttle.attach(throttlepin, 900,2100);
  servo_yaw.attach(yawpin, 1000, 2000);
  servo_gcs1.attach(gcs1pin, 1000, 2000); 
  servo_gcs2.attach(gcs2pin, 1000, 2000); 
}


void 
AP_Program::update(servo_out &ServoChan)
{
  static uint16_t gcsMode = gcs_mode_change;
  static uint16_t mode    = 900;       
  uint16_t currMode       = _radio->chan8(); // channels[7]; channel number 8  ID 7
  // Priority loop checks for mode change from controller first
  if(currMode != mode)                     
  {
    // mode change from controller has happened - update mode
    mode = currMode;
  } 
  else
  {
    // no mode change from controller - check for mode change from gcs  
    if(gcs_mode_change != gcsMode)
    {
      mode = gcs_mode_change;
    }
    gcsMode = gcs_mode_change;
  }
  
  
  ServoChan.chan8  = _radio->chan8();
  
  mode = gcs_mode_change;
  
  if(FLIGHT_MODE(mode) != IGNORE)
    flight_mode = FLIGHT_MODE(mode);

  // FLIGHT MODE
  _ahrs->set_flightmode(flight_mode);

  // ALTITUDE AND HEIGHT FILTER
  AP_hgtfilter_aq.update();

  // TECS
  // Make the Airspeed Estimate equal to estimated speed
  AP_tecs.update_50Hz(AP_hgtfilter_aq.altitude(), AP_hgtfilter_aq.climbrate());
  
  #if DEBUG_TECS
  PRINT(AP_tecs.altitude());
  PRINT_TAB();
  PRINT(AP_tecs.climbrate() * 100);
  PRINT_TAB();
  PRINT(AP_hgtfilter_aq.altitude());
  PRINT_TAB();
  PRINT(AP_hgtfilter_aq.climbrate() * 100);
  PRINT_LINE();
  #endif
  
  filtered.altitude  = AP_tecs.altitude();
  filtered.climbrate = AP_tecs.climbrate();
  //filtered.speed     = AP_tecs.speed();

  
  
  switch(flight_mode)
  {
    case MANUAL:
    manual_control(ServoChan);
    break;
    
    case STABILISE:
    stabilised_control(ServoChan);
    break;

    case AUTO:
     switch(do_guided){
      case false: 
        automatic_control(ServoChan);
        break;

      case true:
        //guided_control(ServoChan);
        guided_navigation(ServoChan);
        break;
     }
    break;
  }
  
  #if FREEZE
  Serial.print("Chan ");
  Serial.print(ServoChan.chan1);
  Serial.print(" ");
  Serial.print(ServoChan.chan2);
  Serial.print(" ");
  Serial.print(ServoChan.chan3);
  Serial.print(" ");
  Serial.print(ServoChan.chan4);  
  Serial.print(" ");
  Serial.println(ServoChan.chan8);
  #endif
}


void 
AP_Program::manual_control(servo_out &ServoChan)
{
  /* 
   * populate servo channel
   */
  ServoChan.chan1 = _radio->chan1();
  ServoChan.chan2 = _radio->chan2();
  ServoChan.chan3 = _radio->chan3();  
  ServoChan.chan4 = _radio->chan4();
  
  /* 
   *  write servos
   */
  servo_roll.writeMicroseconds(ServoChan.chan1);
  servo_pitch.writeMicroseconds(ServoChan.chan2);
  servo_throttle.writeMicroseconds(ServoChan.chan3);
  servo_yaw.writeMicroseconds(ServoChan.chan4);
  servo_gcs1.writeMicroseconds(ServoChan.chan5);
  servo_gcs2.writeMicroseconds(ServoChan.chan6);  
}


float 
AP_Program::_spd_scaler(){
  float airspeed;
  float spd_scaler; 
  float ref_speed  = (_parameter->ParameterStorage.list.min_speed + _parameter->ParameterStorage.list.max_speed) * 0.5f;
  airspeed         = constrain_float(_ahrs->airspeed_estimate(), _parameter->ParameterStorage.list.min_speed, _parameter->ParameterStorage.list.max_speed);   
  spd_scaler       = ref_speed/airspeed;

  return spd_scaler;
}


// Wrapped to +/-180 degrees
bool 
AP_Program::degrees_tohold(float &bearing)
{
  
  float chan4     = _radio->chan4();
  float mid_yaw   =  0.5f * (_parameter->ParameterStorage.list.min_yaw_aux + _parameter->ParameterStorage.list.max_yaw_aux);
  float deadzone  = 50;
  float yawmin    = mid_yaw - deadzone;
  float yawmax    = mid_yaw + deadzone;
  
  if(chan4 >= yawmin && chan4 <= yawmax)
    chan4  = mid_yaw;
  else if(chan4 < yawmin)
    chan4  = chan4 + deadzone;
  else if(chan4 > yawmax)
    chan4  = chan4 - deadzone;

  float chan1    = _radio->chan1();
  float mid_roll =  0.5f * (_parameter->ParameterStorage.list.min_roll_aux + _parameter->ParameterStorage.list.max_roll_aux);
  float rollmin  = mid_roll - deadzone;
  float rollmax  = mid_roll + deadzone;

  if(chan1 >= rollmin && chan1 <= rollmax)
    chan1  = mid_roll;
  else if(chan1 < rollmin)
    chan1  = chan1 + deadzone;
  else if(chan1 > rollmax)
    chan1  = chan1 - deadzone;
  
  
  float dem_rud   = map_float(chan4, _parameter->ParameterStorage.list.min_yaw_aux, _parameter->ParameterStorage.list.max_yaw_aux, -60, 60); 
  float dem_roll  = map_float(chan1, _parameter->ParameterStorage.list.min_roll_aux, _parameter->ParameterStorage.list.max_roll_aux, -45, 45); 
  float yaw_error = 0;

  // IF throttle is more than 30 percent and sticks are centered,     hold heading
  // IF throttle is more than 30 percent and sticks are not centered, dont hold heading 
  // IF throttle is less than 30 even thow sticks might be centered,  dont hold any heading
  if(_servo_out.throttle() >= 30)
  {
    if(chan4 == mid_yaw && chan1 == mid_roll)
    {
      if(_hdg_hold == false)
      {
         yaw_dem   = ToDeg(_ahrs->yaw);
         _hdg_hold = true;

         PRINTLN(yaw_dem);
      }  
    }  
    else
    {
      _hdg_hold  = false;
    } 
  }
  else
  {
      _hdg_hold = false;
  }

  if(_hdg_hold == true){
     bearing = yaw_dem;
     return true;
  }
  else{
    return false;
  }
}


float 
AP_Program::roll_stabilise(float demanded_roll,  float _spd_scaler){
  float dem_roll        = demanded_roll;
  float rev_roll_sig    = _parameter->ParameterStorage.list.rev_roll_sig;
  if(fabs(rev_roll_sig) == 1.0f)
    dem_roll          *= rev_roll_sig;
  float _angle_err_r = dem_roll - ToDeg(_ahrs->roll);
  float _meas_rate_r = ToDeg(_ahrs->rollrate);  
  
  return map_float(AP_rollcontroller.servo_out (_angle_err_r, _meas_rate_r, _spd_scaler), -45, 45, _parameter->ParameterStorage.list.min_roll_aux,  _parameter->ParameterStorage.list.max_roll_aux);
}


float 
AP_Program::pitch_stabilise(float demanded_pitch, float _spd_scaler){
  float dem_pitch      = demanded_pitch;
  float  rev_pitch_sig = _parameter->ParameterStorage.list.rev_pitch_sig; 
  if(fabs(rev_pitch_sig) == 1.0f)
    dem_pitch *= rev_pitch_sig;
  float _angle_err_p  = dem_pitch - ToDeg(_ahrs->pitch);
  float _meas_rate_p  = ToDeg(_ahrs->pitchrate); 
  
  return map_float(AP_pitchcontroller.servo_out(_angle_err_p, _meas_rate_p, _spd_scaler), -45, 45, _parameter->ParameterStorage.list.min_pitch_aux, _parameter->ParameterStorage.list.max_pitch_aux);
}


float
AP_Program::throttle_stabilise(float demanded_throttle_chan){
  /*@ SlewRate is the percentage throttle change over the full scale range in one second
   *@ SR = (ΔThrottle/FullScaleRange)/dt
   */
  float current_throttle = demanded_throttle_chan;   
  float slew_rate        =  _parameter->ParameterStorage.list.slew_rate * 0.01f; // in 100 percent
  throttle = current_throttle * slew_rate + throttle * (1 - slew_rate);
  return throttle;
}


float
AP_Program::yaw_stabilise(float rudder_aux,  float aileron_out, float _spd_scaler){
  float rudder_out     = (aileron_out - (0.5f * (_parameter->ParameterStorage.list.min_roll_aux + _parameter->ParameterStorage.list.max_roll_aux))) * _parameter->ParameterStorage.list.rudder_mix; 
  float  rev_rud_sig   = _parameter->ParameterStorage.list.rev_yaw_sig;
  if(fabs(rev_rud_sig) == 1.0f){
    rudder_out    *= rev_rud_sig;
  }
  rudder_out      += rudder_aux;
  rudder_out       = constrain_float(rudder_out, _parameter->ParameterStorage.list.min_yaw_aux, _parameter->ParameterStorage.list.max_yaw_aux);
  
  float yaw_damper = map_float(AP_yawcontroller.servo_out(_spd_scaler), -45, 45, -200, 200);

  if(filtered.altitude > 5)
  {
    return rudder_out + yaw_damper; 
  }
  else
  {  
    float hold_bearing = ToDeg(_ahrs->yaw);   
    if(degrees_tohold(hold_bearing))
    {
      float yaw_error      = wrap_180(hold_bearing - ToDeg(_ahrs->yaw));
      float rev_rud_sig    = _parameter->ParameterStorage.list.rev_yaw_sig;
      if(fabs(rev_rud_sig) == 1.0f)
       yaw_error *= rev_rud_sig; 
      float _angle_err_y  = yaw_error; // replace this with yaw_error
      float _meas_rate_y  = ToDeg(_ahrs->yawrate); 
      return map_float(AP_steercontroller.servo_out(_angle_err_y, _meas_rate_y), -45, 45, _parameter->ParameterStorage.list.min_yaw_aux, _parameter->ParameterStorage.list.max_yaw_aux);  
    }
    else
    {
      return rudder_out + yaw_damper;
    }
  }
}


void 
AP_Program::stabilised_control(servo_out &ServoChan)
{ 
  /*@ Initial setup
   */
   uint64_t now     = micros();
   float    DT      = (now  - update_stab_usec) * 1e-6f;
   update_stab_usec = now;
   if(DT > 1.0f){
   throttle = _radio->chan3();
   DT       = 0.02f;
   }
   
  /*@ Roll
   */
  float dem_roll    = map_float(_radio->chan1(), _parameter->ParameterStorage.list.min_roll_aux, _parameter->ParameterStorage.list.max_roll_aux, -_parameter->ParameterStorage.list.max_roll_deg, _parameter->ParameterStorage.list.max_roll_deg); 
  
  /*@ Pitch
   */
  float dem_pitch   = map_float(_radio->chan2(), _parameter->ParameterStorage.list.min_pitch_aux, _parameter->ParameterStorage.list.max_pitch_aux, -_parameter->ParameterStorage.list.max_pitch_deg, _parameter->ParameterStorage.list.max_pitch_deg); 
    
  /*@ Throttle
   */
  float dem_throttle = _radio->chan3(); 

  /*@ Rudder
   */
  float dem_rudder  = _radio->chan4();


  /*@ Servo Channels 
   */
  ServoChan.chan1 = roll_stabilise(dem_roll, _spd_scaler());
  ServoChan.chan2 = pitch_stabilise(dem_pitch, _spd_scaler());
  ServoChan.chan3 = throttle_stabilise(dem_throttle); 
  ServoChan.chan4 = yaw_stabilise(dem_rudder, ServoChan.chan1, _spd_scaler());

  /*
  Serial.print(F("Attitude"));Serial.print(" ");Serial.print(dem_roll);Serial.print(" "); Serial.println(dem_pitch);
  Serial.print(F("Chans"));Serial.print(" ");Serial.print(ServoChan.chan1);Serial.print(" "); Serial.println(ServoChan.chan2);
  */
  
  /*@ Servo PWM 
   */
  servo_roll.writeMicroseconds(ServoChan.chan1);
  servo_pitch.writeMicroseconds(ServoChan.chan2);
  servo_throttle.writeMicroseconds(ServoChan.chan3);
  servo_yaw.writeMicroseconds(ServoChan.chan4);
  servo_gcs1.writeMicroseconds(ServoChan.chan5);
  servo_gcs2.writeMicroseconds(ServoChan.chan6);
}


float 
AP_Program::roll_navigation(float demanded_roll,  float _spd_scaler){
  float dem_roll        = demanded_roll;
  float rev_roll_sig    = _parameter->ParameterStorage.list.rev_roll_sig;
  if(fabs(rev_roll_sig) == 1.0f)
    dem_roll          *= rev_roll_sig;
  float _angle_err_r = dem_roll - ToDeg(_ahrs->roll);
  float _meas_rate_r = ToDeg(_ahrs->rollrate);  
  
  return map_float(AP_rollcontroller.servo_out (_angle_err_r, _meas_rate_r, _spd_scaler), -45, 45, _parameter->ParameterStorage.list.min_roll_aux,  _parameter->ParameterStorage.list.max_roll_aux);
}


float 
AP_Program::pitch_navigation (float demanded_pitch, float _spd_scaler){
  float dem_pitch      = demanded_pitch;
  float  rev_pitch_sig = _parameter->ParameterStorage.list.rev_pitch_sig; 
  if(fabs(rev_pitch_sig) == 1.0f)
    dem_pitch *= rev_pitch_sig;
  float _angle_err_p  = dem_pitch - ToDeg(_ahrs->pitch);
  float _meas_rate_p  = ToDeg(_ahrs->pitchrate); 
  
  return map_float(AP_pitchcontroller.servo_out(_angle_err_p, _meas_rate_p, _spd_scaler), -45, 45, _parameter->ParameterStorage.list.min_pitch_aux, _parameter->ParameterStorage.list.max_pitch_aux);
}


float
AP_Program::throttle_navigation(float demanded_throttle_chan){
  float current_throttle = demanded_throttle_chan;   
  float slew_rate        =  _parameter->ParameterStorage.list.slew_rate * 0.01f; // Provision to use a different slew rate in navigation
  throttle = current_throttle * slew_rate + throttle * (1 - slew_rate);          // Provision to use a different static variable for nav throttle
  return throttle;
}


float
AP_Program::yaw_navigation(float aileron_out, float _spd_scaler){
  float rudder_out     = (aileron_out - (0.5f * (_parameter->ParameterStorage.list.min_roll_aux + _parameter->ParameterStorage.list.max_roll_aux))) * _parameter->ParameterStorage.list.rudder_mix; 
  float  rev_rud_sig   = _parameter->ParameterStorage.list.rev_yaw_sig;
  if(fabs(rev_rud_sig) == 1.0f){
    rudder_out    *= rev_rud_sig;
  }
  rudder_out     += 0.5f * (_parameter->ParameterStorage.list.min_yaw_aux + _parameter->ParameterStorage.list.max_yaw_aux);
  rudder_out      = constrain_float(rudder_out, _parameter->ParameterStorage.list.min_yaw_aux, _parameter->ParameterStorage.list.max_yaw_aux);  
  float yaw_damper = map_float(AP_yawcontroller.servo_out(_spd_scaler), -45, 45, -200, 200);
  return rudder_out + yaw_damper; 
}


float 
AP_Program::roll_nav_stickmix(float aileron_out){
  float aileron_midpoint  = 0.5f * (_parameter->ParameterStorage.list.min_roll_aux +  _parameter->ParameterStorage.list.max_roll_aux);
  float delta_output      = aileron_out   - aileron_midpoint;
  float aileron_override  = _radio->chan1();
  float delta_override    = aileron_override - aileron_midpoint; 
  float direct_mixing     = 0.15;

  aileron_out   = aileron_midpoint  + delta_output * (1-direct_mixing) + delta_override * direct_mixing; 
  
  return aileron_out;
}


float
AP_Program::pitch_nav_stickmix(float elevator_out){
  float elevator_midpoint   = 0.5f * (_parameter->ParameterStorage.list.min_pitch_aux +  _parameter->ParameterStorage.list.max_pitch_aux);
  float delta_output_elev   = elevator_out - elevator_midpoint;
  float elevator_override   = _radio->chan2();
  float delta_override_elev = elevator_override - elevator_midpoint;
  
  float direct_mixing    = 0.15;

  elevator_out  = elevator_midpoint + delta_output_elev * (1-direct_mixing) + delta_override_elev * direct_mixing; 

  return elevator_out;
}

Location AP_Program::one_km_waypoint(Location prevWP, float bearing){
  float offset = 1000;          // one km offset 
  float Radius = 6371000;       // earth's radius
  float d      = offset/Radius; // angular radian distance

  float lat    = sin(ToRad(prevWP.lat)) * cos(d) + cos(ToRad(prevWP.lat)) * sin(d) * cos(bearing);
  lat          = asin(lat);     // in radians

  float lon1    = sin(bearing) * sin(d) * cos(ToRad(prevWP.lat));
  float lon2    = cos(d) - sin(ToRad(prevWP.lat)) * sin(lat);  
  float lon    = ToRad(prevWP.lon) + atan2(lon1, lon2); // radians

  // changed to degrees
  lat          = ToDeg(lat);
  lon          = ToDeg(lon);             
  lon          = wrap_180(lon); // wrap 180
  
  Location WB;
  WB.lat       = lat;
  WB.lon       = lon;
 
  return WB;
}

bool AP_Program::distance_to_completion_check(const Location &WPB){
   Location WPAir;
   _ahrs->get_position(WPAir);
   boolean distance_check = static_cast<float>(_NE_distance(WPAir, WPB).length()) <= static_cast<float>(WPB.rad);
   return distance_check;
}

bool AP_Program::cross_the_finish_line_check(const Location &WPA, const Location &WPB){
   Location WPAir;
   _ahrs->get_position(WPAir);
   Vector2f BA   = _NE_distance(WPB, WPA);
   Vector2f BAir = _NE_distance(WPB, WPAir);
   BA.normalise();
   BAir.normalise(); 
   boolean finishline_check   = (BA * BAir) < 0;   
   return finishline_check;
}


void AP_Program::automatic_control(servo_out &ServoChan){
  
  /*
   * Home WayPoint Should have an altitude of 100m
   * Get the current mission sequence
   * WPA corresponds to prevWP
   * WPB corresponds to next mission item - sequence
   * Cross the finish line logic to increment mission sequence
   */  
   uint64_t now = micros();
   float DT     = (now - auto_usec) * 1e-6f;
   auto_usec    = now;

   static float bearing;
   //static bool  gps_landing          = false;
   static float _last_altitude       = 100;

   if(DT > 1.0f){
    is_autolanding   = false;
    is_autotakeoff   = false;
    throttle         = 1000;
    bearing          = _ahrs->yaw;
   }

  /*@ WayPoints Error
   *@ Probably report for no waypoints ~ do nothing
   */
   if(_waypoint->count.total == 0){
    PRINTLN(F("No Valid Waypoints"));
    _flagship = 1;  
    //return;
   }

   switch(mission_complete)
   {
     case true:
          // loiter around home::
          // OLD
          // loiter_waypoint(ServoChan, _waypoint->WayPoint[seq], _last_altitude);
          // NEW
          loiter_waypoint(ServoChan, _waypoint->homeWP, _last_altitude);
          break;


     case false:
          byte next_seq = 0;
          seq == (_waypoint->count.total - 1) ?   next_seq = 0 : next_seq = seq + 1; // check if we have finished the mission
        
          // NEW
          if(_last_seq != seq){
            WPA = _waypoint->read(seq);
            WPB = _waypoint->read(next_seq);
            _last_seq = seq; 
          }

          // OLD
          /*
          Location WPA = _waypoint->WayPoint[seq];
          WPB          = _waypoint->WayPoint[next_seq];
          */
          
          if(next_seq == 0){
            _last_altitude                   = WPA.alt; 
            _waypoint->WayPoint[next_seq].alt= WPA.alt;
            _waypoint->WayPoint[next_seq].rad= WPA.rad;
          } 
               
          /*
           *@ Check if we have an autotakeoff command on WP1
           *@ if not then reset autotakeoff, takecares of new waypoint uploads
           */
          if((next_seq == 1) && (WPB.cmd == FLIGHT_TAKEOFF))
          {
            if(_ahrs->groundspeed_vector().length() > 4.5)
            {
              if(_ahrs->gps().have_gps() == true)
              {
                if(gps_takeoff_enabled == false)
                {
                  // GPS heading
                  bearing      = ToRad(wrap_180(_ahrs->gps().heading()));
                  #if !DEBUG
                  PRINT("GPS ");
                  PRINT(ToDeg(bearing));
                  PRINT_LINE();
                  #endif
                  gps_takeoff_enabled = true;
                }

                float yawDeg = wrap_360(ToDeg(_ahrs->yaw));
                if((wrap_180(_ahrs->gps().heading() - yawDeg) > 45.0f) && 
                (_ahrs->_wind_estimate().length() < _ahrs->groundspeed_vector().length() * 0.8f) &&
                (_ahrs->groundspeed_vector().length() > 3.1f)){
                  PRINT(F("GPS and Compass Covariance Error"));
                }
              }
            }

            Location WP = WPB;
            WPB         = one_km_waypoint(WPA, bearing);
            WPB.alt     = WP.alt;
            WPB.rad     = WP.rad;
            WPB.cmd     = WP.cmd;
            is_autotakeoff = true;
          }
          else
          {
            is_autotakeoff = false;
          }
                            
          /*@ If the last waypoint is for landing 
           *@ set autolanding flag to true
           *@ incase an upload happens then reset autolanding to enable a sequence update
           */
          if((next_seq == (_waypoint->count.total - 1)) && (WPB.cmd  == FLIGHT_LAND))
          {
            is_autolanding = true;
          }
          else
          {
            is_autolanding = false;
          }


          /*@ Autolanding checks
           */
          if(is_autolanding){
            // Force landing altitude to be zero for the TECS controller
            WPB.alt     = 0;

            if(distance_to_completion_check(WPB)    == false &&
               cross_the_finish_line_check(WPA,WPB) == false  &&
               _ahrs->altitude_estimate() >= 5){
               WPB.cmd = FLIGHT_LAND_APPROACH;
               }
               

            if(distance_to_completion_check(WPB) == true ||
            cross_the_finish_line_check(WPA,WPB) == true || 
            AP_tecs.flaring() == true)
            {
              Location copyWP = WPB;
              WPB         = one_km_waypoint(WPA, bearing);
              WPB.alt     = copyWP.alt;
              WPB.rad     = copyWP.rad;
              WPB.cmd     = copyWP.cmd;
            }
            else
            {
              // check speed | last true bearing before we went to the logic above
              if(_ahrs->groundspeed_vector().length() > 4.5 &&  (_ahrs->gps().have_gps() == true))
              {
                bearing  = ToRad(wrap_180(_ahrs->gps().heading()));            
                float yawDeg = wrap_360(ToDeg(_ahrs->yaw));
                if((wrap_180(_ahrs->gps().heading() - yawDeg) > 45.0f) && (_ahrs->_wind_estimate().length() < _ahrs->groundspeed_vector().length() * 0.8f)){
                  PRINT(F("GPS and Compass Covariance Error"));
                }                      
              }
              else 
              {
                bearing = _ahrs->yaw;
              }  
            }           
          }
                         
          /*@ L1
           *@ update_waypoiny(previous_waypoint, next_waypoint) 
           */          
          float accel = AP_l1.update_waypoint(WPA, WPB);
          if(next_seq == 0){
            accel     = AP_l1.update_loiter(_waypoint->WayPoint[next_seq], _waypoint->WayPoint[next_seq].rad);
          }
          
          /*@ Demanded Equivalent Airspeed
           */
          float EAS_demanded = _parameter->ParameterStorage.list.cru_speed;   
          if(is_autotakeoff == true)
          {
            EAS_demanded = 0.8f * _parameter->ParameterStorage.list.cru_speed;  // Provision for user to define takeoff speed or rotation speed
          }
          else if(is_autolanding == true)
          {
            EAS_demanded = _parameter->ParameterStorage.list.land_speed;; // Provision for user to define landing speed
          }
          else
          {
            EAS_demanded  = _parameter->ParameterStorage.list.cru_speed;
          }
        
          /*@ TECS 
           *@ update_pitcg_throttle(demanded_altitude, demanded_speed, navigation command)
           */
          AP_tecs.update_pitch_throttle(WPB.alt, EAS_demanded, WPB.cmd);        

        
          /*@ Demanded Roll
           */
          float dem_roll     = atan2(accel, 9.81f);         
          dem_roll           = ToDeg(dem_roll);
          dem_roll           = constrain_float(dem_roll, -_parameter->ParameterStorage.list.L1_bank, _parameter->ParameterStorage.list.L1_bank);
          if((is_autotakeoff == true)&& (filtered.altitude <= 25))
          {
            dem_roll = constrain_float(dem_roll, -5.0f, 5.0f); 
          } 
          else if((is_autolanding == true) && (filtered.altitude <= 25.0f))
          {
            dem_roll = constrain_float(dem_roll, -7.0f, 7.0f); 
          }
            
               
          /*@ TECS Demanded Pitch
           */
          float _demanded_pitch = constrain_float(ToDeg(AP_tecs.pitch_dem()),_parameter->ParameterStorage.list.AUTO_pitchmin, _parameter->ParameterStorage.list.AUTO_pitchmax);
          if((is_autotakeoff == true)     && (_ahrs->airspeed_estimate() < 9)) // Provision to add takeoff speed
          {
            _demanded_pitch = constrain_float(_demanded_pitch, 1.0f, 3.0f); 
          } 
          else if((is_autotakeoff == true)&& (_ahrs->airspeed_estimate() > 9))
          {
            _demanded_pitch = constrain_float(_demanded_pitch, 3.0f, 15.0f);        
          }

                        
          /*@ TECS Demanded Throttle
           */
          float dem_thr = AP_tecs.throttle_dem();           
          float Thrmaxf = _parameter->ParameterStorage.list.max_thr_aux;
          float Thrminf = _parameter->ParameterStorage.list.min_thr_aux;
          float Throutf = dem_thr * (Thrmaxf - Thrminf) + Thrminf;
          if((is_autotakeoff == true))
          {
            Throutf = Thrmaxf; 
          } 
                                   
          /*@ Outputs
           */
          float aileron_out  = roll_navigation(dem_roll, _spd_scaler());
          float elevator_out = pitch_navigation(_demanded_pitch, _spd_scaler());
          float throttle_out = throttle_navigation(Throutf);     
          float rudder_out   = yaw_navigation(aileron_out, _spd_scaler());
        
          /*@ Stick Mixing
           */       
          aileron_out   = roll_nav_stickmix(aileron_out); 
          elevator_out  = pitch_nav_stickmix(elevator_out);        
          
          /*@ Servo Channels
           */
          ServoChan.chan1 = aileron_out;
          ServoChan.chan2 = elevator_out;
          ServoChan.chan3 = throttle_out;
          ServoChan.chan4 = rudder_out;       
            
          /*@ Servo PWM Write
           */
          servo_roll.writeMicroseconds(ServoChan.chan1);
          servo_pitch.writeMicroseconds(ServoChan.chan2);
          servo_throttle.writeMicroseconds(ServoChan.chan3);
          servo_yaw.writeMicroseconds(ServoChan.chan4);
          servo_gcs1.writeMicroseconds(ServoChan.chan5);
          servo_gcs2.writeMicroseconds(ServoChan.chan6);
          
          /*
           * Finished current sequence using cross the finish line logic or waypoint reached
           */
           Location WPAir;
           _ahrs->get_position(WPAir);
           boolean distance_check     = static_cast<float>(_NE_distance(WPAir, WPB).length()) <= static_cast<float>(WPB.rad);
           Vector2f BA   = _NE_distance(WPB, WPA);
           Vector2f BAir = _NE_distance(WPB, WPAir);
           BA.normalise();
           BAir.normalise(); 
           boolean finishline_check   = (BA * BAir) < 0; 
        
           /*
            * If not landing increment sequence
            * If landing then:
            * 1. Check flag for finished landing
            * 2. if finished landing return loop after DT
            * 3. if DT is greater than 1 then reset autolanding and reset flag for finished landing
            * 4. meet conditions but in autotakeoff check altitude before increasing
            */
           if(is_autotakeoff)
           {
             if(filtered.altitude >= WPB.alt)
             {
              // do takeoff checks and increment for finished takeoff
              is_autotakeoff = false; // finished takeoff unless otherwise it will increment sequence after passed altitude
              seq++;
             }    
           }
           else if(is_autolanding)
           {
             // do landing checks and do finished landing procedure
             // Best thing is to check if aircraft is flying before writing if it is then dont write
             if(!_ahrs->isflying())
             {
                // shut down aircraft
             }
           }
           else if(distance_check || finishline_check)
           {
              // do normal waypoint finished checks
              seq++; 
           }
        
           
           /*
            * We have finished mission and we are now at home. 
            * we can either decide to: 
            * 1. loiter here unlimited by setting boolean loiter to true and resetting seq to 0 
            * 2. restart mission by setting seq to 0 and setting boolean loiter to false ! 
            */
           if(seq >= _waypoint->count.total)
           {
             mission_complete  = true;
             seq   = 0;
           } 
           break; 

   }
}



void  AP_Program::loiter_waypoint(servo_out &ServoChan, Location WPLoiter, float altitude){
  /*@ L1
   *@ update_loiter(loiter_waypoint, loiter_radius in metres) 
   */
  float accel = AP_l1.update_loiter(WPLoiter, WPLoiter.rad);

  /*@ TECS 
   *@ update_pitcg_throttle(demanded_altitude, demanded_speed, Flight_Command)
   */
  AP_tecs.update_pitch_throttle(altitude, _parameter->ParameterStorage.list.cru_speed, FLIGHT_LOITER_UNLIM);

  /*@ Stabilise
   *@ Roll 
   */
  float dem_roll     = atan2(accel, 9.81f);
  float rev_roll_sig = _parameter->ParameterStorage.list.rev_roll_sig; 
  dem_roll           = ToDeg(dem_roll);
  dem_roll           = constrain_float(dem_roll, -_parameter->ParameterStorage.list.L1_bank, _parameter->ParameterStorage.list.L1_bank); 
  if(fabs(rev_roll_sig) == 1.0f)
  {
    dem_roll        *= rev_roll_sig;
  }
  float _angle_err_r = dem_roll - ToDeg(_ahrs->roll);
  float _meas_rate_r = ToDeg(_ahrs->rollrate);

  /*@ Stabilise
   *@ Pitch
   */
  float _demanded_pitch = AP_tecs.pitch_dem();
  float  rev_pitch_sig  = _parameter->ParameterStorage.list.rev_pitch_sig;
  float dem_pitch       = ToDeg(_demanded_pitch);
  if(fabs(rev_pitch_sig) == 1.0f)
    dem_pitch *= rev_pitch_sig;
  float _angle_err_p  = dem_pitch - ToDeg(_ahrs->pitch);
  float _meas_rate_p  = ToDeg(_ahrs->pitchrate); 


  /*@ TECS 
   *@ Throttle 
   */
  float dem_thr = AP_tecs.throttle_dem();           
  float Thrmaxf = _parameter->ParameterStorage.list.max_thr_aux;
  float Thrminf = _parameter->ParameterStorage.list.min_thr_aux;
  float Throutf = dem_thr * (Thrmaxf - Thrminf) + Thrminf;

  
  /*@ Scalar
   *@ Airspeed scaler
   */
  float airspeed;
  float _spd_scaler; 
  float ref_speed  = static_cast<float>((_parameter->ParameterStorage.list.min_speed + _parameter->ParameterStorage.list.max_speed) * 0.5f);
  airspeed         = constrain_float(_ahrs->airspeed_estimate(), _parameter->ParameterStorage.list.min_speed, _parameter->ParameterStorage.list.max_speed);   
  _spd_scaler      = ref_speed/airspeed;   

  
  /*@ Outputs
   *@ Populate
   */
  float aileron_out  = map_float(AP_rollcontroller.servo_out (_angle_err_r, _meas_rate_r, _spd_scaler), -45, 45, _parameter->ParameterStorage.list.min_roll_aux,  _parameter->ParameterStorage.list.max_roll_aux);
  float elevator_out = map_float(AP_pitchcontroller.servo_out(_angle_err_p, _meas_rate_p, _spd_scaler), -45, 45, _parameter->ParameterStorage.list.min_pitch_aux, _parameter->ParameterStorage.list.max_pitch_aux);;
  float throttle_out = Throutf;
  float rev_rud_sig = _parameter->ParameterStorage.list.rev_yaw_sig;
  float rudder_out   = (ServoChan.chan1 - static_cast<float>(0.5f * (_parameter->ParameterStorage.list.min_roll_aux + _parameter->ParameterStorage.list.max_roll_aux))) * _parameter->ParameterStorage.list.rudder_mix; 
  if(fabs(rev_rud_sig) == 1.0f)
  {
    rudder_out *= rev_rud_sig;
  }
  rudder_out     += static_cast<float>(0.5f * (_parameter->ParameterStorage.list.min_yaw_aux + _parameter->ParameterStorage.list.max_yaw_aux));
  rudder_out      = constrain_float(rudder_out, _parameter->ParameterStorage.list.min_yaw_aux, _parameter->ParameterStorage.list.max_yaw_aux);
  

  // Direct Mixing Algorithm
  float aileron_midpoint    = static_cast<float>(0.5f * (_parameter->ParameterStorage.list.min_roll_aux +  _parameter->ParameterStorage.list.max_roll_aux));
  float delta_output        = aileron_out   - aileron_midpoint;
  float aileron_override    = _radio->chan1();
  float delta_override      = aileron_override - aileron_midpoint;

  float elevator_midpoint   = static_cast<float>(0.5f * (_parameter->ParameterStorage.list.min_pitch_aux +  _parameter->ParameterStorage.list.max_pitch_aux));
  float delta_output_elev   = elevator_out - elevator_midpoint;
  float elevator_override   = _radio->chan2();
  float delta_override_elev = elevator_override - elevator_midpoint;
  
  float direct_mixing    = 0.15;

  aileron_out   = aileron_midpoint  + delta_output * (1-direct_mixing) + delta_override * direct_mixing; 
  elevator_out  = elevator_midpoint + delta_output_elev * (1-direct_mixing) + delta_override_elev * direct_mixing; 

  
  /*@ Servo Channels
   *@ Populate
   */
  ServoChan.chan1 = aileron_out;
  ServoChan.chan2 = elevator_out;
  ServoChan.chan3 = throttle_out;
  ServoChan.chan4 = rudder_out;

    
  /*@ Servo Function 
   *@ Servo PWM
   */
  servo_roll.writeMicroseconds(ServoChan.chan1);
  servo_pitch.writeMicroseconds(ServoChan.chan2);
  servo_throttle.writeMicroseconds(ServoChan.chan3);
  servo_yaw.writeMicroseconds(ServoChan.chan4);
  servo_gcs1.writeMicroseconds(ServoChan.chan5);
  servo_gcs2.writeMicroseconds(ServoChan.chan6);
}



/*@ After X seconds of loiter go home 
 *@ Before timer start check distance
 */
void  AP_Program::guided_navigation(servo_out &ServoChan){
  uint64_t now = micros();
  float DT     = (now - guided_usec) * 1e-6f;
  guided_usec  = now;

  static float lapsed_seconds = 0;
  static bool  start_count    = false;
  static Location _last_WPB;
  static Location WPA;
 
  /*@ Initial setup
   */
  if(DT > 1.0f){
    start_count  = false;
    _last_WPB.zero(); 
    _ahrs->get_position(WPA);
  }

  // For a change in waypointy
  if(WPB.lat != _last_WPB.lat || 
     WPB.lon != _last_WPB.lon ||
     WPB.alt != _last_WPB.alt || 
     WPB.rad != _last_WPB.rad){
     start_count = false;
     _ahrs->get_position(WPA);
  }
  _last_WPB = WPB;  

  if(start_count == true){
    lapsed_seconds += DT; 
  }else{
    lapsed_seconds  = 0;
  }

  // nimefika kwenye waypoint
  if(distance_to_completion_check(WPB) ||
     cross_the_finish_line_check(WPA, WPB)){
    start_count = true;
  }
     
  /*@ L1 loiter
   */
  if(lapsed_seconds >= static_cast<float>(loiter_seconds)){
     WPB.lat  = _waypoint->WayPoint[0].lat;
     WPB.lon  = _waypoint->WayPoint[0].lon;
  }
  float accel = AP_l1.update_loiter(WPB, WPB.rad);


  /*@ Demanded Speed
   */
  float EAS_demanded = _parameter->ParameterStorage.list.cru_speed;   


  /*@ TECS 
   *@ update_pitcg_throttle(demanded_altitude, demanded_speed, navigation command)
   */
  AP_tecs.update_pitch_throttle(WPB.alt, EAS_demanded, WPB.cmd);        


  /*@ Demanded Roll
   */
  float dem_roll =  constrain_float(ToDeg(atan2(accel, 9.81f)), -_parameter->ParameterStorage.list.L1_bank, _parameter->ParameterStorage.list.L1_bank);

       
  /*@ TECS Demanded Pitch
   */
  float _demanded_pitch = constrain_float(ToDeg(AP_tecs.pitch_dem()), _parameter->ParameterStorage.list.AUTO_pitchmin, _parameter->ParameterStorage.list.AUTO_pitchmax);

                
  /*@ TECS Demanded Throttle
   */
  float dem_thr = AP_tecs.throttle_dem() * (float)(_parameter->ParameterStorage.list.max_thr_aux - _parameter->ParameterStorage.list.min_thr_aux) + (float)(_parameter->ParameterStorage.list.min_thr_aux);           

                           
  /*@ Outputs
   */
  float aileron_out  = roll_navigation(dem_roll, _spd_scaler());
  float elevator_out = pitch_navigation(_demanded_pitch, _spd_scaler());
  float throttle_out = throttle_navigation(dem_thr);     
  float rudder_out   = yaw_navigation(aileron_out, _spd_scaler());

  /*@ Stick Mixing
   */       
  aileron_out   = roll_nav_stickmix(aileron_out); 
  elevator_out  = pitch_nav_stickmix(elevator_out);        
  
  /*@ Servo Channels
   */
  ServoChan.chan1 = aileron_out;
  ServoChan.chan2 = elevator_out;
  ServoChan.chan3 = throttle_out;
  ServoChan.chan4 = rudder_out;       
    
  /*@ Servo PWM Write
   */
  servo_roll.writeMicroseconds(ServoChan.chan1);
  servo_pitch.writeMicroseconds(ServoChan.chan2);
  servo_throttle.writeMicroseconds(ServoChan.chan3);
  servo_yaw.writeMicroseconds(ServoChan.chan4);
  servo_gcs1.writeMicroseconds(ServoChan.chan5);
  servo_gcs2.writeMicroseconds(ServoChan.chan6);
}

