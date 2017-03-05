// Receiver
#include <mavlink.h>
#define HEADER_LEN 8
#define UAV     1

uint16_t led_counter = 0;

uint32_t _last_time = 0;
float    rssi;
float    packets_lost = 0;
float    packets_not_lost = 0;
void setup() {
  // main serial port to send to GCS
  Serial.begin(115200);

  // main serial port to send and receive from aircraft
  Serial1.begin(57600);

  pinMode(13, OUTPUT);
  _last_time = millis();
}

void loop() {
  uint32_t now = millis();
  if((now - _last_time) >= 1000){
     packets_not_lost *= 0.8f;
     packets_lost     *= 0.8f;
  }
  rssi = (packets_not_lost/ (packets_not_lost + packets_lost)) * 100;

  
  mav_stream(&Serial,  &Serial1);
  mav_stream(&Serial1, &Serial);
  blink_led();
}

void blink_led()
{
  led_counter++;
  if(led_counter == 2000 && digitalRead(13) == HIGH){
    digitalWrite(13, LOW);
  }else if(led_counter == 4000 && digitalRead(13) == LOW){
    led_counter = 0;
    digitalWrite(13, HIGH);
  }
}
/*
 * Receive on Serial0
 * Unpack Message
 * Pack   Message
 * Send to Serial1
 * SendPort - Port We Send to
 * RecPort  - Port we receive on
 */
void mav_stream(HardwareSerial *RecPort, HardwareSerial *SendPort)
  {
   mavlink_message_t msg;
   mavlink_status_t status;
   while(RecPort->available())
   {
    uint8_t read_bytes = RecPort->read();  
    if(mavlink_parse_char(MAVLINK_COMM_0, read_bytes, &msg, &status))
    {
      switch(msg.msgid)
      {
         case MAVLINK_MSG_ID_HEARTBEAT:{    
            uint32_t custom_mode     = mavlink_msg_heartbeat_get_custom_mode(&msg);
            uint8_t  type            = mavlink_msg_heartbeat_get_type(&msg);
            uint8_t  autopilot       = mavlink_msg_heartbeat_get_autopilot(&msg);
            uint8_t  base_mode       = mavlink_msg_heartbeat_get_base_mode(&msg);
            uint8_t  system_status   = mavlink_msg_heartbeat_get_system_status(&msg);  
            uint8_t  mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);

            mavlink_message_t msg_t;
            uint8_t buf[MAVLINK_MSG_ID_HEARTBEAT_LEN+ HEADER_LEN];  
             
            mavlink_msg_heartbeat_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            type, 
            autopilot, 
            base_mode, 
            custom_mode, 
            system_status);      
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len);
            }
            break;


        case MAVLINK_MSG_ID_ATTITUDE:{
            uint32_t time_boot_ms   = mavlink_msg_attitude_get_time_boot_ms(&msg);
            float roll              = mavlink_msg_attitude_get_roll(&msg);
            float pitch             = mavlink_msg_attitude_get_pitch(&msg);
            float yaw               = mavlink_msg_attitude_get_yaw(&msg);
            float rollspeed         = mavlink_msg_attitude_get_rollspeed(&msg);
            float pitchspeed        = mavlink_msg_attitude_get_pitchspeed(&msg);
            float yawspeed          = mavlink_msg_attitude_get_yawspeed(&msg);

            //Serial.print("ROLL   ");
            //Serial.println(roll);
            
            mavlink_message_t msg_t;
            uint8_t buf[MAVLINK_MSG_ID_ATTITUDE_LEN + HEADER_LEN];  
            
            mavlink_msg_attitude_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            time_boot_ms, 
            roll, 
            pitch, 
            yaw, 
            rollspeed, 
            pitchspeed, 
            yawspeed);     
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len);            
            }
            break;  


        case MAVLINK_MSG_ID_COMMAND_LONG:{
            float param1              = mavlink_msg_command_long_get_param1(&msg);
            float param2              = mavlink_msg_command_long_get_param2(&msg);
            float param3              = mavlink_msg_command_long_get_param3(&msg);
            float param4              = mavlink_msg_command_long_get_param4(&msg);
            float param5              = mavlink_msg_command_long_get_param5(&msg);
            float param6              = mavlink_msg_command_long_get_param6(&msg);
            float param7              = mavlink_msg_command_long_get_param7(&msg);
            uint16_t command          = mavlink_msg_command_long_get_command(&msg);
            uint8_t target_system     = mavlink_msg_command_long_get_target_system(&msg);
            uint8_t target_component  = mavlink_msg_command_long_get_target_component(&msg);
            uint8_t confirmation      = mavlink_msg_command_long_get_confirmation(&msg);

            mavlink_message_t msg_t;
            uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN + HEADER_LEN];  
            
            mavlink_msg_command_long_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            target_system, 
            target_component, 
            command, 
            confirmation, 
            param1, 
            param2, 
            param3, 
            param4, 
            param5, 
            param6, 
            param7);   
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len);              
            }
            break;


        case MAVLINK_MSG_ID_RAW_IMU:{
            uint64_t time_usec = mavlink_msg_raw_imu_get_time_usec(&msg);
            int16_t xacc       = mavlink_msg_raw_imu_get_xacc(&msg);
            int16_t yacc       = mavlink_msg_raw_imu_get_yacc(&msg);
            int16_t zacc       = mavlink_msg_raw_imu_get_zacc(&msg);
            int16_t xgyro      = mavlink_msg_raw_imu_get_xgyro(&msg);
            int16_t ygyro      = mavlink_msg_raw_imu_get_ygyro(&msg);
            int16_t zgyro      = mavlink_msg_raw_imu_get_zgyro(&msg);
            int16_t xmag       = mavlink_msg_raw_imu_get_xmag(&msg);
            int16_t ymag       = mavlink_msg_raw_imu_get_ymag(&msg);
            int16_t zmag       = mavlink_msg_raw_imu_get_zmag(&msg);   

            mavlink_message_t msg_t;
            uint8_t buf[MAVLINK_MSG_ID_RAW_IMU_LEN + HEADER_LEN];  
            
            mavlink_msg_raw_imu_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            time_usec, 
            xacc, 
            yacc, 
            zacc, 
            xgyro, 
            ygyro, 
            zgyro, 
            xmag, 
            ymag, 
            zmag);  
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len);  
            }
            break;


         case MAVLINK_MSG_ID_GPS_RAW_INT:{
            uint64_t time_usec   = mavlink_msg_gps_raw_int_get_time_usec(&msg);
            int32_t lat          = mavlink_msg_gps_raw_int_get_lat(&msg);
            int32_t lon          = mavlink_msg_gps_raw_int_get_lon(&msg);
            int32_t alt          = mavlink_msg_gps_raw_int_get_alt(&msg);
            uint16_t eph         = mavlink_msg_gps_raw_int_get_eph(&msg);
            uint16_t epv         = mavlink_msg_gps_raw_int_get_epv(&msg);
            uint16_t vel         = mavlink_msg_gps_raw_int_get_vel(&msg);
            uint16_t cog         = mavlink_msg_gps_raw_int_get_cog(&msg);
            uint8_t fix_type     = mavlink_msg_gps_raw_int_get_fix_type(&msg);
            uint8_t satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);

            mavlink_message_t msg_t;
            uint8_t buf[MAVLINK_MSG_ID_GPS_RAW_INT_LEN + HEADER_LEN];  
             
            mavlink_msg_gps_raw_int_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            time_usec, 
            fix_type, 
            lat, 
            lon, 
            alt, 
            eph, 
            epv, 
            vel, 
            cog, 
            satellites_visible);    
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len);                      
            }
            break;


         case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:{
            uint32_t time_usec    = mavlink_msg_servo_output_raw_get_time_usec(&msg);
            uint16_t servo1_raw   = mavlink_msg_servo_output_raw_get_servo1_raw(&msg);
            uint16_t servo2_raw   = mavlink_msg_servo_output_raw_get_servo2_raw(&msg);
            uint16_t servo3_raw   = mavlink_msg_servo_output_raw_get_servo3_raw(&msg);
            uint16_t servo4_raw   = mavlink_msg_servo_output_raw_get_servo4_raw(&msg);
            uint16_t servo5_raw   = mavlink_msg_servo_output_raw_get_servo5_raw(&msg);
            uint16_t servo6_raw   = mavlink_msg_servo_output_raw_get_servo6_raw(&msg);
            uint16_t servo7_raw   = mavlink_msg_servo_output_raw_get_servo7_raw(&msg);
            uint16_t servo8_raw   = mavlink_msg_servo_output_raw_get_servo8_raw(&msg);
            uint8_t port          = mavlink_msg_servo_output_raw_get_port(&msg);      

            mavlink_message_t msg_t;
            uint8_t buf[MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + HEADER_LEN];  
             
            mavlink_msg_servo_output_raw_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            time_usec, 
            port, 
            servo1_raw, 
            servo2_raw, 
            servo3_raw, 
            servo4_raw, 
            servo5_raw, 
            servo6_raw, 
            servo7_raw, 
            servo8_raw);  
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len);
            }
            break;


         case MAVLINK_MSG_ID_VFR_HUD:{          
            float airspeed    = mavlink_msg_vfr_hud_get_airspeed(&msg);
            float groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
            int16_t heading   = mavlink_msg_vfr_hud_get_heading(&msg);
            uint16_t throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
            float alt         = mavlink_msg_vfr_hud_get_alt(&msg);
            float climb       = mavlink_msg_vfr_hud_get_climb(&msg);

            mavlink_message_t msg_t;
            uint8_t  buf[MAVLINK_MSG_ID_VFR_HUD_LEN + HEADER_LEN];     //  PAX 3 + 8 = 11 bytes                          
            mavlink_msg_vfr_hud_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            airspeed, 
            groundspeed, 
            heading, 
            throttle,
            alt, 
            climb);
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len); 
            }
            break;


        case MAVLINK_MSG_ID_SYS_STATUS:{
            uint32_t onboard_control_sensors_present  = mavlink_msg_sys_status_get_onboard_control_sensors_present(&msg);
            uint32_t onboard_control_sensors_enabled  = mavlink_msg_sys_status_get_onboard_control_sensors_enabled(&msg);
            uint32_t onboard_control_sensors_health   = mavlink_msg_sys_status_get_onboard_control_sensors_health(&msg);
            uint16_t load                             = mavlink_msg_sys_status_get_load(&msg);
            uint16_t voltage_battery                  = mavlink_msg_sys_status_get_voltage_battery(&msg);
            int16_t current_battery                   = mavlink_msg_sys_status_get_current_battery(&msg);
            uint16_t drop_rate_comm                   = static_cast<uint16_t>(rssi);//mavlink_msg_sys_status_get_drop_rate_comm(&msg);
            uint16_t errors_comm                      = mavlink_msg_sys_status_get_errors_comm(&msg);
            uint16_t errors_count1                    = mavlink_msg_sys_status_get_errors_count1(&msg);
            uint16_t errors_count2                    = mavlink_msg_sys_status_get_errors_count2(&msg);
            uint16_t errors_count3                    = mavlink_msg_sys_status_get_errors_count3(&msg);
            uint16_t errors_count4                    = mavlink_msg_sys_status_get_errors_count4(&msg);
            int8_t battery_remaining                  = mavlink_msg_sys_status_get_battery_remaining(&msg);

            mavlink_message_t msg_t;
            uint8_t buf[MAVLINK_MSG_ID_SYS_STATUS_LEN + HEADER_LEN];  
            mavlink_msg_sys_status_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            onboard_control_sensors_present, 
            onboard_control_sensors_enabled, 
            onboard_control_sensors_health, 
            load, 
            voltage_battery, 
            current_battery, 
            battery_remaining, 
            drop_rate_comm, 
            errors_comm, 
            errors_count1, 
            errors_count2, 
            errors_count3, 
            errors_count4);   
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len);
            }
            break;
         
         case MAVLINK_MSG_ID_MISSION_COUNT:{
             uint16_t count           = mavlink_msg_mission_count_get_count(&msg);
             uint8_t target_system    = mavlink_msg_mission_count_get_target_system(&msg);
             uint8_t target_component = mavlink_msg_mission_count_get_target_component(&msg);

             mavlink_message_t msg_t;
             uint8_t buf[MAVLINK_MSG_ID_MISSION_COUNT_LEN + HEADER_LEN];  
             mavlink_msg_mission_count_pack(
             UAV, 
             MAV_COMP_ID_ALL, 
             &msg_t,
             target_system, 
             target_component, 
             count);      
             uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
             SendPort->write(buf,len);
             }
             break;
         
         case MAVLINK_MSG_ID_MISSION_ITEM:{         
             float param1 = mavlink_msg_mission_item_get_param1(&msg);
             float param2 = mavlink_msg_mission_item_get_param2(&msg);
             float param3 = mavlink_msg_mission_item_get_param3(&msg);
             float param4 = mavlink_msg_mission_item_get_param4(&msg);
             float x                  = mavlink_msg_mission_item_get_x(&msg);
             float y                  = mavlink_msg_mission_item_get_y(&msg);
             float z                  = mavlink_msg_mission_item_get_z(&msg);
             uint16_t seq             = mavlink_msg_mission_item_get_seq(&msg);
             uint16_t command         = mavlink_msg_mission_item_get_command(&msg);
             uint8_t target_system    = mavlink_msg_mission_item_get_target_system(&msg);
             uint8_t target_component = mavlink_msg_mission_item_get_target_component(&msg);
             uint8_t frame            = mavlink_msg_mission_item_get_frame(&msg);
             uint8_t current          = mavlink_msg_mission_item_get_current(&msg);
             uint8_t autocontinue     = mavlink_msg_mission_item_get_autocontinue(&msg);

             mavlink_message_t msg_t;
             uint8_t buf[MAVLINK_MSG_ID_MISSION_ITEM_LEN + HEADER_LEN];  
             mavlink_msg_mission_item_pack(
             UAV, 
             MAV_COMP_ID_ALL, 
             &msg_t,
             target_system, 
             target_component, 
             seq, 
             frame, 
             command, 
             current, 
             autocontinue, 
             param1, 
             param2, 
             param3, 
             param4, 
             x, 
             y, 
             z);       
             uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
             SendPort->write(buf,len);             
             }
             break;


         case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:{
             uint8_t target_system    = mavlink_msg_mission_request_list_get_target_system(&msg);
             uint8_t target_component = mavlink_msg_mission_request_list_get_target_component(&msg);

             mavlink_message_t msg_t;
             uint8_t buf[MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN + HEADER_LEN];  
             mavlink_msg_mission_request_list_pack(
             UAV, 
             MAV_COMP_ID_ALL, 
             &msg_t,
             target_system, 
             target_component);         
             uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
             SendPort->write(buf,len);
             }
             break;

         case MAVLINK_MSG_ID_MISSION_REQUEST:{
             uint16_t seq             = mavlink_msg_mission_request_get_seq(&msg);
             uint8_t target_system    = mavlink_msg_mission_request_get_target_system(&msg);
             uint8_t target_component  = mavlink_msg_mission_request_get_target_component(&msg);
             
             mavlink_message_t msg_t;
             uint8_t buf[MAVLINK_MSG_ID_MISSION_REQUEST_LEN + HEADER_LEN];  
             mavlink_msg_mission_request_pack(
             UAV, 
             MAV_COMP_ID_ALL, 
             &msg_t,
             target_system, 
             target_component, 
             seq);          
             uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
             SendPort->write(buf,len);          
             }
             break;

        case MAVLINK_MSG_ID_MISSION_ACK:{
             uint8_t target_system    = mavlink_msg_mission_ack_get_target_system(&msg);
             uint8_t target_component = mavlink_msg_mission_ack_get_target_component(&msg);
             uint8_t type             = mavlink_msg_mission_ack_get_type(&msg);

             mavlink_message_t msg_t;
             uint8_t buf[MAVLINK_MSG_ID_MISSION_ACK_LEN + HEADER_LEN];  
             mavlink_msg_mission_ack_pack(
             UAV, 
             MAV_COMP_ID_ALL, 
             &msg_t,
             target_system, 
             target_component, 
             type);         
             uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
             SendPort->write(buf,len);                     
             }
             break;

        
         case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:{
             uint8_t target_system    = mavlink_msg_param_request_list_get_target_system(&msg);
             uint8_t target_component = mavlink_msg_param_request_list_get_target_component(&msg);
             
             mavlink_message_t msg_t;
             uint8_t buf[MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN + HEADER_LEN];     //  PAX 3 + 8 = 11 bytes 
             mavlink_msg_param_request_list_pack(
             UAV, 
             MAV_COMP_ID_ALL, 
             &msg_t,
             target_system, 
             target_component);            
             uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
             SendPort->write(buf,len); 
             }
             break;

         
         case MAVLINK_MSG_ID_PARAM_SET:{         
             char     param_id[16];            
             uint8_t target_system    = mavlink_msg_param_set_get_target_system(&msg);
             uint8_t target_component = mavlink_msg_param_set_get_target_component(&msg);
             mavlink_msg_param_set_get_param_id(&msg, param_id);
             float param_value        = mavlink_msg_param_set_get_param_value(&msg);         
             uint8_t param_type       = mavlink_msg_param_set_get_param_type(&msg);

             mavlink_message_t msg_t;
             uint8_t  buf[MAVLINK_MSG_ID_PARAM_SET_LEN + HEADER_LEN];     //  PAX 3 + 8 = 11 bytes 
             mavlink_msg_param_set_pack(
             UAV, 
             MAV_COMP_ID_ALL, 
             &msg_t,
             target_system, 
             target_component, 
             param_id, 
             param_value, 
             param_type);
             uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
             SendPort->write(buf,len);             
             }
             break;

             
         case MAVLINK_MSG_ID_PARAM_VALUE:{
            float param_value     = mavlink_msg_param_value_get_param_value(&msg);
            uint16_t param_count  = mavlink_msg_param_value_get_param_count(&msg);
            uint16_t param_index  = mavlink_msg_param_value_get_param_index(&msg);
            char param_id[16]; 
            mavlink_msg_param_value_get_param_id(&msg,param_id);
            uint8_t param_type  = mavlink_msg_param_value_get_param_type(&msg);      

            mavlink_message_t msg_t;
            uint8_t  buf[MAVLINK_MSG_ID_PARAM_VALUE_LEN + HEADER_LEN];     //  PAX 3 + 8 = 11 bytes 
            
            mavlink_msg_param_value_pack(
            UAV, 
            MAV_COMP_ID_ALL, 
            &msg_t,
            param_id,
            param_value, 
            param_type, 
            param_count, 
            param_index);
            uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
            SendPort->write(buf,len);                      
            }
            break;
         
         case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:{
             uint16_t chan1 = mavlink_msg_rc_channels_override_get_chan1_raw(&msg);
             uint16_t chan2 = mavlink_msg_rc_channels_override_get_chan2_raw(&msg);
             uint16_t chan3 = mavlink_msg_rc_channels_override_get_chan3_raw(&msg);
             uint16_t chan4 = mavlink_msg_rc_channels_override_get_chan4_raw(&msg);
             uint16_t chan5 = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);
             uint16_t chan6 = mavlink_msg_rc_channels_override_get_chan6_raw(&msg);
             uint16_t chan7 = mavlink_msg_rc_channels_override_get_chan7_raw(&msg);
             uint16_t chan8 = mavlink_msg_rc_channels_override_get_chan8_raw(&msg);
             uint8_t target_system = mavlink_msg_rc_channels_override_get_target_system(&msg);
             uint8_t target_component= mavlink_msg_rc_channels_override_get_target_component(&msg);

             mavlink_message_t msg_t;
             uint8_t  buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN + HEADER_LEN];     //  PAX 3 + 8 = 11 bytes             
             mavlink_msg_rc_channels_override_pack(
             UAV, 
             MAV_COMP_ID_ALL, 
             &msg_t,
             target_system,
             target_component,
             chan1,
             chan2,
             chan3,
             chan4,
             chan5,
             chan6,
             chan7,
             chan8);
             uint16_t len = mavlink_msg_to_send_buffer(buf,&msg_t);         
             SendPort->write(buf,len);  
             }
             break;
      }
    }
   }

   packets_lost += status.packet_rx_drop_count;

   if(status.packet_rx_drop_count == 0){
   packets_not_lost++;
   }
 }
