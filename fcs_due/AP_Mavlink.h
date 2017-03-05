#pragma once
class AP_Mavlink
{
  private:
    HardwareSerial* Port;
    AP_Storage  &_AP_params;
    AP_AHRS     &_AP_ahrs;
    AP_GPS      &_AP_gps;
    AP_INS      &_AP_ins;
    AP_Compass  &_AP_compass;
    AP_Airspeed &_AP_airspeed;
    AP_Baro     &_AP_baro;
    AP_WayPoint &_AP_waypoint;

    AP_Program      &_AP_program;
    AP_BatteryVolts &_AP_batteryvolts;

    mavlink_message_t msg;
    //uint8_t           buf[128];// PAX 26 + 8 = 34 bytes -> 128 Bytes probably a time

    void stream_10Hz();
    void stream_3Hz();
    void stream_1Hz();
    byte _stream_speed_10Hz;
    byte _stream_speed_3Hz;
    byte _stream_speed_1Hz;

  public:
    AP_Mavlink(
    AP_Storage &ap_st, AP_AHRS &ap_ah, AP_GPS &ap_gp,
    AP_INS &ap_in, AP_Compass &ap_co, AP_Airspeed &ap_ar,
    AP_Baro &ap_ba, AP_WayPoint &ap_wp, AP_Program &ap_pgm, 
    AP_BatteryVolts &ap_bat):
    _AP_params(ap_st),
    _AP_ahrs(ap_ah),
    _AP_gps(ap_gp),
    _AP_ins(ap_in),
    _AP_compass(ap_co),
    _AP_airspeed(ap_ar),
    _AP_baro(ap_ba),
    _AP_program(ap_pgm),
    _AP_batteryvolts(ap_bat),
    _stream_speed_10Hz(0),
    _stream_speed_3Hz(0),
    _stream_speed_1Hz(0),
    _AP_waypoint(ap_wp)
    {};

    void initialise(HardwareSerial *_Port)
    {
      Port = _Port;
    }

    void sendstream(); // execute in a 50Hz loop

    void readstream()
    {
      while (Port->available())
      {
        #if !MAVLINK_OLD_IMPLEMENTATION
        //mavlink_message_t msg;
        mavlink_status_t status;
        #endif
        
        uint8_t read_bytes = Port->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, read_bytes, &msg, &status))
        {
          switch (msg.msgid)
          {
            case MAVLINK_MSG_ID_HEARTBEAT: {
              }
              break;

            case MAVLINK_MSG_ID_COMMAND_LONG:{
                if((mavlink_msg_command_long_get_target_system(&msg) == UAV) &&
                   (mavlink_msg_command_long_get_target_component(&msg) == MAV_COMP_ID_SYSTEM_CONTROL)){

                    switch(mavlink_msg_command_long_get_command(&msg)){
                      case MAV_CMD_NAV_LOITER_TIME:{
                        /*
                        MAV_CMD_NAV_LOITER_TIME  Loiter around this MISSION for X seconds
                        Mission Param #1  Seconds (decimal)
                        Mission Param #2  Empty
                        Mission Param #3  Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
                        Mission Param #4  Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
                        Mission Param #5  Latitude
                        Mission Param #6  Longitude
                        Mission Param #7  Altitude
                        */ 
                        uint8_t confirmation =  mavlink_msg_command_long_get_confirmation(&msg);
                        float lat    = mavlink_msg_command_long_get_param5(&msg);
                        float lon    = mavlink_msg_command_long_get_param6(&msg);
                        uint16_t alt = static_cast<uint16_t>(mavlink_msg_command_long_get_param7(&msg));
                        uint8_t rad  = static_cast<uint8_t>(mavlink_msg_command_long_get_param3(&msg));
                        uint16_t time_seconds = static_cast<uint16_t>(mavlink_msg_command_long_get_param1(&msg));
                       
                        Location current_wp, target_wp; 
                        target_wp.lat = lat;
                        target_wp.lon = lon;
                        target_wp.alt = alt;
                        target_wp.rad = rad;
                        _AP_program.ahrs().get_position(current_wp);
                        _AP_program.guided(true, current_wp, target_wp, time_seconds);
                        send_command_ack((uint16_t)MAV_CMD_NAV_LOITER_TIME, (uint8_t)MAV_RESULT_ACCEPTED);
                        }
                        break;

                      case MAV_CMD_MISSION_START:{
                        _AP_program.Mission_Start();
                        send_command_ack((uint16_t)MAV_CMD_MISSION_START, (uint8_t)MAV_RESULT_ACCEPTED);
                        }
                        break;
                        
                      }
                   }
              }
              break;

            // Consider Facing-out out due to above method
            case MAVLINK_MSG_ID_SET_GLOBAL_POSITION_SETPOINT_INT:{
              uint8_t unique_id = mavlink_msg_set_global_position_setpoint_int_get_coordinate_frame(&msg);
              if(unique_id == 100 || unique_id == 101){
                float lat    = static_cast<float>(mavlink_msg_set_global_position_setpoint_int_get_latitude(&msg) * 1e-7f);
                float lon    = static_cast<float>(mavlink_msg_set_global_position_setpoint_int_get_longitude(&msg) * 1e-7f);
                uint16_t alt = static_cast<uint16_t>(mavlink_msg_set_global_position_setpoint_int_get_altitude(&msg) * 1e-3);
                uint8_t rad  = static_cast<uint8_t>(mavlink_msg_set_global_position_setpoint_int_get_yaw(&msg) * 1e-2);
                uint8_t time_seconds = mavlink_msg_global_position_setpoint_int_get_coordinate_frame(&msg);

                Location current_wp, target_wp; 
                target_wp.lat = lat;
                target_wp.lon = lon;
                target_wp.alt = alt;
                target_wp.rad = rad;
                _AP_program.ahrs().get_position(current_wp);
                _AP_program.guided(true, current_wp, target_wp, time_seconds);
              }
              
              }
              break;

            /*
               Receiiving Mission Count and then Sending Mission Items
            */
            case MAVLINK_MSG_ID_MISSION_COUNT: {
                if (mavlink_msg_mission_count_get_target_system(&msg) == UAV) {
                  _AP_waypoint.count.total = static_cast<AP_u8>(mavlink_msg_mission_count_get_count(&msg));   // need to save it
                  _AP_waypoint.count.save();
                  send_mission_request(0);
                  //_AP_program.guided(false); // since i am sending a command to start mission, i will say dont do guided from there ~ will continue with guided until a command to start a mission has been received
                }
              }
              break;

            case MAVLINK_MSG_ID_MISSION_ITEM: {
                 if (mavlink_msg_mission_item_get_target_system(&msg) == UAV)
                 {
                   uint16_t _seq = mavlink_msg_mission_item_get_seq(&msg);
                   _AP_waypoint.update(mavlink_msg_mission_item_get_x(&msg), mavlink_msg_mission_item_get_y(&msg), static_cast<AP_u16>(mavlink_msg_mission_item_get_z(&msg)), static_cast<AP_u8>(mavlink_msg_mission_item_get_param1(&msg)), static_cast<AP_u8>(_seq), static_cast<AP_u8>(mavlink_msg_mission_item_get_command(&msg)));

                   // This function writes to eeprom takes about 3.3ms to write
                   // Best thing is to check if aircraft is flying before writing if it is then dont write
                   if(!_AP_program.ahrs().isflying())
                   {
                     _AP_waypoint.write(_seq);
                   }
                  
                   if (_seq < (_AP_waypoint.count.total - 1)) {
                     send_mission_request(_seq + 1);
                    } else if (_seq == (_AP_waypoint.count.total - 1)) {
                     send_mission_ack((uint8_t)MAV_MISSION_ACCEPTED);
                    } else if (_seq >= MAX_WAYPOINTS) {
                     send_mission_ack((uint8_t)MAV_MISSION_NO_SPACE);
                    } else {
                     send_mission_ack((uint8_t)MAV_MISSION_INVALID);
                   }
                }
              }
              break;

            /*
                Sending Requested Mission Item List
            */
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
                 if (mavlink_msg_mission_request_list_get_target_system(&msg) == UAV)
                  send_mission_count(_AP_waypoint.count.total);
              }
              break;

            case MAVLINK_MSG_ID_MISSION_REQUEST: {
                if (mavlink_msg_mission_request_get_target_system(&msg) == UAV)
                  send_mission_item(static_cast<AP_u8>(mavlink_msg_mission_request_get_seq(&msg)));
              }
              break;

            /*@ Onboard Parameters 
             */
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                 SEND_MAV_DATA  = true;
                 SendParamList(UAV);
              }
              break;

            case MAVLINK_MSG_ID_PARAM_SET: {
                 if (mavlink_msg_param_set_get_target_system(&msg) == UAV)
                 {
                  char param_id[16];
                  float param_value = mavlink_msg_param_set_get_param_value(&msg);
                  mavlink_msg_param_set_get_param_id(&msg, param_id);
                  UpdateStorage(UAV, param_id, param_value);
                 }
              }
              break;

            case MAVLINK_MSG_ID_HIL_STATE:
              {
#if HIL_SIM == 1
                hil_ahrs _hil_ahrs;
                _hil_ahrs.roll     = mavlink_msg_hil_state_get_roll(&msg);      // rad
                _hil_ahrs.pitch    = mavlink_msg_hil_state_get_pitch(&msg);     // rad
                _hil_ahrs.yaw      = mavlink_msg_hil_state_get_yaw(&msg);       // rad
                _hil_ahrs.rollrate = mavlink_msg_hil_state_get_rollspeed(&msg); // rad/sec
                _hil_ahrs.pitchrate = mavlink_msg_hil_state_get_pitchspeed(&msg); // rad/sec
                _hil_ahrs.yawrate  = mavlink_msg_hil_state_get_yawspeed(&msg);  // rad/sec
                _hil_ahrs.xacc     = (mavlink_msg_hil_state_get_xacc(&msg) * 0.001f) * 9.81f;       // mg to m/s2
                _hil_ahrs.yacc     = (mavlink_msg_hil_state_get_yacc(&msg) * 0.001f) * 9.81f;       // mg to m/s2
                _hil_ahrs.zacc     = (mavlink_msg_hil_state_get_zacc(&msg) * 0.001f) * 9.81f;       // mg to m/s2

                /*
                  hil_gps _hil_gps;
                  _hil_gps.location.x= static_cast<float>(mavlink_msg_hil_state_get_lat(&msg) * 1e-7);
                  _hil_gps.location.y= static_cast<float>(mavlink_msg_hil_state_get_lon(&msg) * 1e-7);
                  _hil_gps.velocity.x= static_cast<float>( mavlink_msg_hil_state_get_vx(&msg) * 1e-2);
                  _hil_gps.velocity.y= static_cast<float>( mavlink_msg_hil_state_get_vy(&msg) * 1e-2);
                  _hil_gps.velocity.z= static_cast<float>( mavlink_msg_hil_state_get_vz(&msg) * 1e-2);
                  _hil_gps.altitude  = static_cast<float>(mavlink_msg_hil_state_get_alt(&msg) * 1e-3);
                  _hil_gps.time_usec = mavlink_msg_hil_state_get_time_usec(&msg);
                */
                float lat = static_cast<float>(mavlink_msg_hil_state_get_lat(&msg) * 1e-7);
                float lon = static_cast<float>(mavlink_msg_hil_state_get_lon(&msg) * 1e-7);
                float vx  = static_cast<float>( mavlink_msg_hil_state_get_vx(&msg) * 1e-2);
                float vy  = static_cast<float>( mavlink_msg_hil_state_get_vy(&msg) * 1e-2);
                float vz  = static_cast<float>( mavlink_msg_hil_state_get_vz(&msg) * 1e-2);
                float alt = static_cast<float>(mavlink_msg_hil_state_get_alt(&msg) * 1e-3);

                _AP_ahrs.setHil();
                _AP_ins.setHil(_hil_ahrs.rollrate, _hil_ahrs.pitchrate, _hil_ahrs.yawrate, _hil_ahrs.xacc, _hil_ahrs.yacc, _hil_ahrs.zacc);
                _AP_compass.setHil(_hil_ahrs.roll, _hil_ahrs.pitch, _hil_ahrs.yaw);
                _AP_gps.setHil(lat, lon, alt, vector3f(vx, vy, vz));
                _AP_baro.setHil(alt);
#endif
              }
              break;

            case MAVLINK_MSG_ID_VFR_HUD: {
              
#if HIL_SIM == 1
                //_AP_airspeed.setHil(mavlink_msg_vfr_hud_get_airspeed(&msg));
                _AP_gps.setHilHeading(mavlink_msg_vfr_hud_get_heading(&msg));  // 0 -360 in degress
#endif
              }
              break;

            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
                _servo_out.chan5 = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);
                _servo_out.chan6 = mavlink_msg_rc_channels_override_get_chan6_raw(&msg);
              }
              break;
          }
        }
      }
    }

    void send_hb(boolean &Send_Allowed)
{
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t  msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_HEARTBEAT_LEN + HEADER_LEN];     // Pax 9 + 8 = 17bytes
    
      mavlink_msg_heartbeat_pack(
        UAV, 
        MAV_COMP_ID_ALL, 
        &msg,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_GENERIC,
        _AP_ahrs.get_flightmode(),    // base mode
        MAV_MODE_PREFLIGHT,          // custom mode
        MAV_STATE_STANDBY);

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      if (Send_Allowed) {
        Port->write(buf, len);
      }
    }

    void sendParameter(byte uav_id, const char param_id[16], float param_value, uint8_t param_type, uint16_t param_index)
    {
      uint8_t       buf[MAVLINK_MSG_ID_PARAM_VALUE_LEN + 8];
      mavlink_msg_param_value_pack(
       uav_id,  
       MAV_COMP_ID_ALL, 
       (&msg), 
       param_id, 
       param_value, 
       param_type, 
       sizeof(_parameter_list_t)/4, 
       param_index
       );  
      
      uint16_t len = mavlink_msg_to_send_buffer(buf,(&msg)); 
     
      Port->write(buf,len);
    }

    void UpdateStorage(const byte UAV_ID, const char param_id[16], float param_value)
    {  
      char param_buffer[16];
      union _f_un
      {
        float val;
        uint8_t buffer[4];
      };
      union _f_un param_un;
      uint8_t parameter_index = 0;
      
      while(parameter_index < sizeof(_parameter_list_t)/4)
      {
        strcpy_P(param_buffer, (char*)pgm_read_word(&(param_table[parameter_index])));      
        
        // check the returned char array against the received parameter id
        if(strcmp(param_id, param_buffer) == 0)
        {
          param_un.val  = param_value; 
         
          for(int y = 0; y < 4; y++ )
          { 
             EEPROM.write(3000  +  y  +  (parameter_index  *  4), param_un.buffer[y]);                 // write  value to storage
             _AP_params.ParameterStorage.paramBuffer[y + (parameter_index*4)] = param_un.buffer[y];    // updates global parameter
          }      
          for(int y = 0; y < 4; y++ )
          {
             param_un.buffer[y] = EEPROM.read(3000  +  y  +  (parameter_index * 4));        // read from eeprom and send the read parameter and not the received one. THis makes sure that the value has been updated                                                                      // 4ms increment in time
          }
          sendParameter(UAV_ID, param_buffer, param_un.val, MAV_VAR_FLOAT, parameter_index);      
          break;
        }
        parameter_index++;
      }
    }

    void SendParamList(byte UAV_ID)
    {
      union _f_un{
        float   val;
        uint8_t buffer[4];
      };
      union _f_un param_un;                                                            // only one copy is created instead of millions in the while loop..
      char  parameter_id[16];
      byte  parameter_index = 0;
      
      while(parameter_index < sizeof(_parameter_list_t)/4)                             // this is the number of parameters
      {      
        strcpy_P(parameter_id, (char*)pgm_read_word(&(param_table[parameter_index]))); // copy parameter id from flash into parameter_id variable
        
        for(int y = 0; y < 4; y++ )
        {
           param_un.buffer[y] = _AP_params.ParameterStorage.paramBuffer[y  +  (parameter_index  *  4)];    // read paramter from eeprom and send
        }
        
        sendParameter(UAV_ID, parameter_id, param_un.val, MAV_VAR_FLOAT, parameter_index);  
        
        parameter_index++;    
        
        delay(100);  // 10Hz
      }  
      parameter_index = 0;   
    }    

    void send_euler(uint32_t &loop_lapse, boolean &Send_Allowed)
    {
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t  msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_ATTITUDE_LEN + HEADER_LEN];     // PAX 28 + 8 = 36bytes

      mavlink_msg_attitude_pack(
        UAV, MAV_COMP_ID_IMU, &msg,
        loop_lapse,
        _AP_ahrs.roll,
        _AP_ahrs.pitch,
        _AP_ahrs.yaw,  // true heading as calculated by dcm
        _AP_ahrs.rollrate,
        _AP_ahrs.pitchrate,
        _AP_ahrs.yawrate);

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      if (Send_Allowed) {
        Port->write(buf, len);
      }
    }


    void send_imuraw(uint32_t &loop_lapse, boolean &Send_Allowed)
    {
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t  msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_RAW_IMU_LEN + HEADER_LEN];     // PAX 26 + 8 = 34 bytes

      mavlink_msg_raw_imu_pack(
        UAV, MAV_COMP_ID_IMU, &msg,
        loop_lapse,
        static_cast<int16_t>(_AP_ins.raw_accel()[0].x),
        static_cast<int16_t>(_AP_ins.raw_accel()[0].y),
        static_cast<int16_t>(_AP_ins.raw_accel()[0].z),
        static_cast<int16_t>(_AP_ins.raw_gyro()[0].x),
        static_cast<int16_t>(_AP_ins.raw_gyro()[0].y),
        static_cast<int16_t>(_AP_ins.raw_gyro()[0].z),
        static_cast<int16_t>(_AP_compass.raw_field().x),
        static_cast<int16_t>(_AP_compass.raw_field().y),
        static_cast<int16_t>(_AP_compass.raw_field().z));

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      if (Send_Allowed) {
        Port->write(buf, len);
      }
    }

    void send_gps(uint32_t &loop_lapse,  boolean &Send_Allowed)
    {

      uint64_t hours   = static_cast<uint64_t>(_AP_gps.hours()   * 1e5);
      uint64_t minutes = static_cast<uint64_t>(_AP_gps.minutes() * 1e3);
      uint64_t seconds = static_cast<uint64_t>(_AP_gps.seconds() * 1e1);
      uint64_t UTC_Time= hours + minutes + seconds;
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t  msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_GPS_RAW_INT_LEN + HEADER_LEN];     //  PAX 30 + 8 = 38 bytes
     
      mavlink_msg_gps_raw_int_pack(
        UAV, 
        MAV_COMP_ID_GPS, 
        &msg,
        UTC_Time,
        _AP_gps.status(),
        static_cast<i32>(_AP_gps.location().lat * 1e7),
        static_cast<i32>(_AP_gps.location().lon * 1e7),
        static_cast<i32>(_AP_gps.altitude()    * 1e3),
        static_cast<u16>(_AP_gps.horizontal_dilution()),
        static_cast<u16>(_AP_gps.vertical_dilution()),
        static_cast<u16>(_AP_gps.groundspeed()  * 1e2),
        static_cast<u16>(_AP_gps.heading()      * 1e2), // COG
        _AP_gps.num_sats());

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      if (Send_Allowed) {
        Port->write(buf, len);
      }
    }

    void send_servo(uint32_t &loop_lapse, servo_out *servoOut, boolean &Send_Allowed)
    {
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + HEADER_LEN];     //  PAX 21 + 8 = 29 bytes

      mavlink_msg_servo_output_raw_pack(
        UAV, MAV_COMP_ID_ALL, &msg,
        loop_lapse     ,
        1              ,
        servoOut->chan1,  // roll
        servoOut->chan2,  // pitch
        servoOut->chan3,  // throttle
        servoOut->chan4,  // yaw
        servoOut->chan5,  // empty for override
        servoOut->chan6,  // empty for overide
        servoOut->chan7,  // empty for override
        servoOut->chan8); // chan 8 - flight mode channel

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      //Serial.print("Aux "); Serial.print(servoOut->chan1); Serial.print(servoOut->chan2); Serial.print(servoOut->chan3); Serial.println(servoOut->chan4);

      if (Send_Allowed) {
        Port->write(buf, len);
      }
    }

    /*@ Send vfr information
    */
    void send_vfr(uint32_t &loop_lapse, servo_out *srv_chan, boolean &Send_Allowed)
    {
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_VFR_HUD_LEN + HEADER_LEN];     //  PAX 21 + 8 = 29 bytes

      mavlink_msg_vfr_hud_pack(
        UAV,
        MAV_COMP_ID_ALL,
        &msg,
        
#if TECS_FILTER
        _AP_program.tecs().speed(),
#else
        _AP_program.ahrs().airspeed_estimate(),
#endif

        _AP_program.ahrs().groundspeed_vector().length(),
        _AP_gps.heading()   ,
        srv_chan->throttle(),

#if TECS_FILTER
        _AP_program.tecs().altitude(),
        _AP_program.tecs().climbrate()); // Climb rate
#else
        _AP_program.aq().altitude(),
        _AP_program.aq().climbrate()); // Climb rate
#endif

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      if (Send_Allowed) {
        Port->write(buf, len);
      }
    }

    void send_status(uint32_t &loop_lapse,  boolean &Send_Allowed)
    {
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_SYS_STATUS_LEN + HEADER_LEN];     //  PAX 21 + 8 = 29 bytes

      uint32_t onboard_control_sensors_present = B00111111; // Current controls
      uint32_t onboard_control_sensors_enabled = B00111111; // All sensors enabled
      uint32_t onboard_control_sensors_health  = B00111111; // All sensors healthy
      uint16_t load               = 50;                     // Maximum usage in percentage of mainloop time
      uint16_t voltage_battery    = _AP_batteryvolts.get_adc() * _AP_params.ParameterStorage.list.PowerModule_Gain * 1000;
      int8_t   battery_remaining  = 0;
      uint16_t drop_rate_comm     = 0;
      uint16_t errors_comm        = 0;
      uint16_t errors_count1      = _AP_ins.healthy(); // healthy 1
      uint16_t errors_count2      = _AP_gps.healthy(); // healthy 1
      uint16_t errors_count3      = _AP_baro.healthy();// healthy 1
      uint16_t errors_count4      = 0;

      mavlink_msg_sys_status_pack(
        UAV,
        MAV_COMP_ID_ALL,
        &msg,
        onboard_control_sensors_present,
        onboard_control_sensors_enabled,
        onboard_control_sensors_health,
        load,
        voltage_battery,
        -1, // no current measurment
        battery_remaining,
        drop_rate_comm,
        errors_comm,
        errors_count1,
        errors_count2,
        errors_count3,
        errors_count4);

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      if (Send_Allowed) {
        Port->write(buf, len);
      }
    }


    void send_mission_request(const uint8_t &seq)
    {
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_MISSION_REQUEST_LEN + HEADER_LEN];     //  PAX 4 + 8 = 12 bytes

      mavlink_msg_mission_request_pack(
        UAV,
        MAV_COMP_ID_ALL,
        &msg,
        GCS,
        MSP,
        seq);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Port->write(buf, len);
    }

    void send_mission_ack(const uint8_t mav_ack)
    {
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_MISSION_ACK_LEN + HEADER_LEN];     //  PAX 3 + 8 = 11 bytes

      mavlink_msg_mission_ack_pack(
        UAV,
        MAV_COMP_ID_ALL,
        &msg,
        GCS,
        MSP,
        mav_ack);

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Port->write(buf, len);
    }

    void send_command_ack(const uint16_t command_id, const uint8_t result){
      uint8_t  buf[MAVLINK_MSG_ID_COMMAND_ACK_LEN + HEADER_LEN];     //  PAX 3 + 8 = 11 bytes
      mavlink_msg_command_ack_pack(
      UAV, 
      MAV_COMP_ID_ALL, 
      &msg,
      command_id, 
      result);
      
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Port->write(buf, len);   
    }

    void send_mission_count(const uint8_t count)
    {
      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_MISSION_COUNT_LEN + HEADER_LEN];     //  PAX 4 + 8 = 11 bytes
     
      mavlink_msg_mission_count_pack(
        UAV,
        MAV_COMP_ID_ALL,
        &msg,
        GCS,
        MSP,
        count);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Port->write(buf, len);
    }

    void send_mission_item(const uint8_t seq)
    {
      if (seq >= MAX_WAYPOINTS)
        return;

      #if MAVLINK_OLD_IMPLEMENTATION
      mavlink_message_t msg;
      #endif
      uint8_t  buf[MAVLINK_MSG_ID_MISSION_ITEM_LEN + HEADER_LEN];     //  PAX 37 + 8 = 43 bytes

      mavlink_msg_mission_item_pack(
        UAV,
        MAV_COMP_ID_ALL,
        &msg,
        GCS,
        MSP,
        _AP_waypoint.WayPoint[seq].seq,
        MAV_FRAME_GLOBAL,
        _AP_waypoint.WayPoint[seq].cmd,
        0,
        1,
        _AP_waypoint.WayPoint[seq].rad,
        255,
        255,
        255,
        _AP_waypoint.WayPoint[seq].lat,
        _AP_waypoint.WayPoint[seq].lon,
        _AP_waypoint.WayPoint[seq].alt);

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Port->write(buf, len);
    }


     void send_global_gps_setpoint()
     {
        #if MAVLINK_OLD_IMPLEMENTATION
        mavlink_message_t msg;
        #endif
        uint8_t buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT_LEN + HEADER_LEN];
   
        mavlink_msg_global_position_setpoint_int_pack(
        UAV,
        MAV_COMP_ID_ALL,
        &msg,
        0,
        static_cast<int32_t>(_AP_program.Set_WP().lat * 1e7) ,
        static_cast<int32_t>(_AP_program.Set_WP().lon * 1e7) ,
        static_cast<int32_t>(_AP_program.Set_WP().alt * 1e3) ,
        static_cast<int16_t>(_AP_program.Set_WP().rad * 1e2)); // actually meant to be yaw
        uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
        if(SEND_MAV_DATA)
        Serial.write(buf,len);
     }
};

void AP_Mavlink::sendstream()
{
  stream_10Hz();
}

// 10Hz Loop
void AP_Mavlink::stream_10Hz()
{
  switch (_stream_speed_10Hz)
  {
    case 0:
      _stream_speed_10Hz++;
      if(!_AP_program.ahrs().isflying())
        send_imuraw(loop_lapse_time, SEND_MAV_DATA);                // 34 Bytes
      break;

    case 1:
      _stream_speed_10Hz++;
      #if HIL_SIM == 0
      if(!_AP_program.ahrs().isflying())
        send_servo(loop_lapse_time, &_servo_out, SEND_MAV_DATA);    // 29 Bytes
      #else
        send_servo(loop_lapse_time, &_servo_out, SEND_MAV_DATA);    // 29 Bytes
      #endif
      break;

    case 2:
      _stream_speed_10Hz++;
      send_euler(loop_lapse_time, SEND_MAV_DATA);                  // 36 Bytes
      break;

    case 3:
      _stream_speed_10Hz++;
      break;

    case 4:
      _stream_speed_10Hz = 0;
      stream_3Hz();
      break;
  }
}


// 5Hz
void AP_Mavlink::stream_3Hz()
{
  switch (_stream_speed_3Hz)
  {
    case 0:
      _stream_speed_3Hz++;
      send_gps(loop_lapse_time,  SEND_MAV_DATA);               // 38 Bytes
      break;

    case 1:
      stream_1Hz();
      _stream_speed_3Hz = 0;
      break;
  }
}


// 1Hz loop
void AP_Mavlink::stream_1Hz()
{
  switch (_stream_speed_1Hz)
  {
    case 0:
      _stream_speed_1Hz++;
      send_hb(SEND_MAV_DATA);
      send_global_gps_setpoint();
      break;

    case 1:
      _stream_speed_1Hz++;
      if(_AP_program.ahrs().isflying()){
        send_servo(loop_lapse_time, &_servo_out, SEND_MAV_DATA);    // 29 Bytes
      }
      break;

    case 2:
      _stream_speed_1Hz++;
      if(_AP_program.ahrs().isflying()){
        send_imuraw(loop_lapse_time, SEND_MAV_DATA);                // 34 Bytes
      }
      break;

    case 3:
      _stream_speed_1Hz++;
      send_vfr(loop_lapse_time, &_servo_out, SEND_MAV_DATA);        // 28 Bytes
      break;

    case 4:
      _stream_speed_1Hz = 0;
      send_status(loop_lapse_time,  SEND_MAV_DATA);
      break;
  }
}
