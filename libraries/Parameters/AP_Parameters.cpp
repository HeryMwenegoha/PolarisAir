#define CHARS_TABLE  1
#include "AP_Parameters.h"

AP_Storage  AP_params;


AP_Storage::AP_Storage()
{
    
};

void
AP_Storage::ReadAll(void)
{
   for(int x = 0; x < (sizeof(_parameter_list_t)); x++ )
   {
      ParameterStorage.paramBuffer[x] = EEPROM.read(3000+x);
   }
   Serial.println(F("Read All Parameters"));
}


void
AP_Storage::WriteAll(void)
{
  ParameterStorage.list.rudder_mix    = 0.3; // Pos 0
  ParameterStorage.list.rev_roll_sig  = 1.0; // Pos 1
  ParameterStorage.list.rev_pitch_sig = 1.0; // Pos 2
  ParameterStorage.list.rev_yaw_sig   = 1.0; // Pos 3
  ParameterStorage.list.rev_thr_sig   = 1.0; // Pos 4  
  
  ParameterStorage.list.roll_kp       = 0.6;  // Pos 5
  ParameterStorage.list.roll_ki       = 0.2;  // Pos 6
  ParameterStorage.list.roll_kd       = 0.02; // Pos 7
  ParameterStorage.list.roll_tau      = 0.5;  // --> 8
  ParameterStorage.list.roll_rmax     = 60;   // --> 9
  ParameterStorage.list.roll_imax     = 15;   // --> 10
  ParameterStorage.list.max_roll_deg  = 45;   // Pos 11 
  ParameterStorage.list.max_roll_aux  = 2000; // Pos 12 
  ParameterStorage.list.min_roll_aux  = 1000; // Pos 13 
  
  ParameterStorage.list.pitch_kp      = 0.75;  // Pos 14 
  ParameterStorage.list.pitch_ki      = 0.25;  // Pos 15 
  ParameterStorage.list.pitch_kd      = 0.05; // Pos 16 
  ParameterStorage.list.pitch_tau     = 0.5;  // --> 17
  ParameterStorage.list.pitch_rmax    = 60;   // --> 18
  ParameterStorage.list.pitch_imax    = 15;   // --> 19 -- . changed from the old value
  ParameterStorage.list.max_pitch_deg = 25;   // Pos 20
  ParameterStorage.list.PTCH2SRV_RLL  = 1;    // --> 21    
  ParameterStorage.list.max_pitch_aux = 2000; // Pos 22
  ParameterStorage.list.min_pitch_aux = 1000; // Pos 23
  
  ParameterStorage.list.steer_kp        = 1.8;  // Pos 24
  ParameterStorage.list.steer_ki        = 0.25;  // Pos 25
  ParameterStorage.list.steer_kd        = 0.005;  // Pos 26
  ParameterStorage.list.steer_tau       = 0.75;  // --> 27
  ParameterStorage.list.steer_rmax      = 90;  // --> 28
  ParameterStorage.list.steer_imax      = 15;  // --> 29
  ParameterStorage.list.YAW2SRV_SLP   = 0.0;  // --> 30
  ParameterStorage.list.YAW2SRV_RLL   = 1.0;  // --> 31
  ParameterStorage.list.YAW_FFRDDRMIX = 0.3;  // --> 32
  ParameterStorage.list.max_yaw_aux   = 2000; // Pos 33
  ParameterStorage.list.min_yaw_aux   = 1000; // Pos 34
  
  ParameterStorage.list.L1_damping    = 0.707;  // --> 35
  ParameterStorage.list.L1_period     = 20;      // --> 36
  ParameterStorage.list.L1_bank       = 45;      // --> 37
  ParameterStorage.list.L1_gravity    = 9.806; // --> 38
  
  ParameterStorage.list.max_thr_aux   = 2000;    // Pos 39
  ParameterStorage.list.min_thr_aux   = 1000;    // Pos 40
  ParameterStorage.list.max_thr_perc  = 100;     // Pos 41
  ParameterStorage.list.min_thr_perc  = 0;       // Pos 42
  ParameterStorage.list.cru_thr       = 50;      // Pos 43
  ParameterStorage.list.slew_rate     = 75;      // Pos 44
 
  ParameterStorage.list.max_speed     = 20;    // --> 45
  ParameterStorage.list.min_speed     = 5;     // --> 46  
  ParameterStorage.list.cru_speed     = 13;    // Pos 47  // Demanded Speed
  ParameterStorage.list.land_speed    = 10;    // Pos 47  // Demanded Speed
  ParameterStorage.list.cru_altitude  = 100;   // Pos 48  // Demanded Altitude
  
  ParameterStorage.list.APPROACH             = 0.0;   // Pos 49  
  ParameterStorage.list.TECS_thr_Proportional= 1.0f;  // Pos 50
  ParameterStorage.list.TECS_thr_Integrator  = 0.1f;
  ParameterStorage.list.TECS_thr_Damping     = 0.5f; // Used to be 0.
  ParameterStorage.list.TECS_thr_TCONST      = 5.0f;
  ParameterStorage.list.TECS_thr_rmax        = 100;
  ParameterStorage.list.TECS_thr_SlewRate    = 100;
  ParameterStorage.list.TECS_thr_land        = 30;
  ParameterStorage.list.TECS_thr_tomax       = 100;
  ParameterStorage.list.AUTO_pitchmax        =  20;
  ParameterStorage.list.AUTO_pitchmin        = -20;
  
  ParameterStorage.list.TECS_land_pitchmax   =  5;
  ParameterStorage.list.TECS_land_pitchmin   = -5;
  ParameterStorage.list.TECS_maxClimbRate    =  5;
  ParameterStorage.list.TECS_maxSinkRate     =  5;
  ParameterStorage.list.TECS_minSinkRate     = 2;
  ParameterStorage.list.TECS_flare_secs      = 5;
  ParameterStorage.list.arspdEnabled    	 = 1;
  ParameterStorage.list.TECS_stallPrevent    = 1; //
  ParameterStorage.list.PowerModule_Gain	 = 0.05;
  
  
  ParameterStorage.list.Accel1_offsetX	 	= 0;
  ParameterStorage.list.Accel1_offsetY	 	= 0;
  ParameterStorage.list.Accel1_offsetZ		= 0;
  ParameterStorage.list.Accel1_lsbX			= 8192; // LSB/G
  ParameterStorage.list.Accel1_lsbY			= 8192; // LSB/G
  ParameterStorage.list.Accel1_lsbZ			= 8192; // LSB/G
  
  
  ParameterStorage.list.Accel2_offsetX	 	= -212;
  ParameterStorage.list.Accel2_offsetY	 	= 74;
  ParameterStorage.list.Accel2_offsetZ		= -13;
  ParameterStorage.list.Accel2_lsbX			= 8353; // LSB/G
  ParameterStorage.list.Accel2_lsbY			= 8326; // LSB/G
  ParameterStorage.list.Accel2_lsbZ			= 8431; // LSB/G
  
  for(int x = 0; x<(sizeof(_parameter_list_t)); x++)
  {
    EEPROM.write(3000+x,ParameterStorage.paramBuffer[x]);
  }
  
  Serial.println(F("Written	all	parameters"));
}


void
AP_Storage::initialise(HardwareSerial *_Port)
{    
  Port = _Port;
  
  // Code once
  //WriteAll();
    
  // Read Parameters
  ReadAll();
  
  Serial.println(F("Parameters	Initialised"));
}


void 
AP_Storage::mavlink_sendParameter(byte uav_id, const char param_id[16], float param_value, uint8_t param_type, uint16_t param_index, mavlink_message_t *msg)
{
  //mavlink_message_t msg;
  uint8_t 		  buf[MAVLINK_MSG_ID_PARAM_VALUE_LEN + 8];
  mavlink_msg_param_value_pack(
   uav_id,  
   MAV_COMP_ID_ALL, 
   (msg), 
   param_id, 
   param_value, 
   param_type, 
   sizeof(_parameter_list_t)/4, 
   param_index
  );  
  
  uint16_t len = mavlink_msg_to_send_buffer(buf,(msg)); 
 
  Port->write(buf,len);
}


void 
AP_Storage::SendParamList(byte UAV_ID, mavlink_message_t *msg)
{
PGM_Stufff pgm_stuff;

  char un[16];
  
  for(int8_t i = 0; i<60; i++){
   strcpy_P(un ,(char *)pgm_read_byte(&GROUP_DATA[i].idx));
  Serial.print((uint8_t)pgm_read_word(&GROUP_DATA[i].idx));
  
 Serial.print(F("  "));
 uint8_t data_type = (uint8_t)pgm_read_word(&GROUP_DATA[i].type);
 Serial.print(data_type);
 Serial.print(F("  "));
 Serial.print((uint8_t)pgm_read_word(&GROUP_DATA[i].pos));
 Serial.print(F("  "));
 Serial.print((uint8_t)pgm_read_word(&GROUP_DATA[i].len));
 Serial.print(F("  "));
 Serial.print((char *)pgm_read_word(&GROUP_DATA[i].id));
 Serial.print(F("  "));
  Serial.print(pgm_stuff.pgm_read_val(0,i));
  Serial.println(F("  "));
  
  }
	
	
	
  union _f_un param_un;                                                            // only one copy is created instead of millions in the while loop..
  char  parameter_id[16];
  byte  parameter_index = 0;
  
  while(parameter_index < sizeof(_parameter_list_t)/4)                             // this is the number of parameters
  {      
    strcpy_P(parameter_id, (char*)pgm_read_word(&(param_table[parameter_index]))); // copy parameter id from flash into parameter_id variable
    
	for(int y = 0; y < 4; y++ )
    {
       param_un.buffer[y] = ParameterStorage.paramBuffer[y  +  (parameter_index  *  4)];    // read paramter from eeprom and send
    }
    
	mavlink_sendParameter(UAV_ID, parameter_id, param_un.val, MAV_VAR_FLOAT, parameter_index, msg);  
    
	parameter_index++;    
    
	delay(100);  // 10Hz
  }  
  parameter_index = 0;   
}


void AP_Storage::UpdateStorage(const byte UAV_ID, const char param_id[16], float param_value, mavlink_message_t *msg)
{  
  char param_buffer[16];
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
         EEPROM.write(3000  +  y  +  (parameter_index  *  4), param_un.buffer[y]);	    // write  value to storage
		 ParameterStorage.paramBuffer[y + (parameter_index*4)] = param_un.buffer[y];    // updates global parameter
      }
	  
	  for(int y = 0; y < 4; y++ )
      {
         param_un.buffer[y] = EEPROM.read(3000  +  y  +  (parameter_index * 4));      	// read from eeprom and send the read parameter and not the received one. THis makes sure that the value has been updated                                                                      // 4ms increment in time
      }
	 	  
      mavlink_sendParameter(UAV_ID, param_buffer, param_un.val, MAV_VAR_FLOAT, parameter_index, msg);      
	  
	  break;
    }
    parameter_index++;
  }
}
  

