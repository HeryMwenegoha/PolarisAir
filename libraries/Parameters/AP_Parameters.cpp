#define CHARS_TABLE  1
#include "AP_Parameters.h"

AP_Storage  AP_params;

AP_Storage::AP_Storage()
{ 
};

/*
union AP_Storage::union_type(uint8_t type)
{
	switch(type)
	{
		case FLOAT:
		return _f_un;
		
		case U8:
		return _f_un;	
	}
}
*/

void
AP_Storage::ReadAll(void)
{
   for(int x = 0; x < (sizeof(_parameter_list_t)); x++ )
   {
      ParameterStorage.paramBuffer[x] = EEPROM.read(3000+x);
   }
   
   //for(int i = 0; i < 65; i++)
   //Serial.println(get_param(i));
   
   Serial.println(F("AP_Parameters::status	Read	All	Parameters"));
}

void 
AP_Storage::print_all(){
	Serial.println(ParameterStorage.list.rudder_mix);
	Serial.println(ParameterStorage.list.rev_roll_sig);
	Serial.println(ParameterStorage.list.rev_pitch_sig);
	Serial.println(ParameterStorage.list.rev_yaw_sig);
	Serial.println(ParameterStorage.list.rev_thr_sig);

	Serial.println(ParameterStorage.list.roll_kp);
	Serial.println(ParameterStorage.list.roll_ki);
	Serial.println(ParameterStorage.list.roll_kd);
	Serial.println(ParameterStorage.list.roll_tau);
	Serial.println(ParameterStorage.list.roll_rmax);
	Serial.println(ParameterStorage.list.roll_imax);
	Serial.println(ParameterStorage.list.max_roll_deg);
	Serial.println(ParameterStorage.list.max_roll_aux);
	Serial.println(ParameterStorage.list.min_roll_aux);
	 
	Serial.println(ParameterStorage.list.pitch_kp);
	Serial.println(ParameterStorage.list.pitch_ki);
	Serial.println(ParameterStorage.list.pitch_kd);
	Serial.println(ParameterStorage.list.pitch_tau);
	Serial.println(ParameterStorage.list.pitch_rmax);
	Serial.println(ParameterStorage.list.pitch_imax);
	Serial.println(ParameterStorage.list.max_pitch_deg);
	Serial.println(ParameterStorage.list.PTCH2SRV_RLL);
	Serial.println(ParameterStorage.list.max_pitch_aux);
	Serial.println(ParameterStorage.list.min_pitch_aux);

	Serial.println(ParameterStorage.list.steer_kp);
	Serial.println(ParameterStorage.list.steer_ki);
	Serial.println(ParameterStorage.list.steer_kd);
	Serial.println(ParameterStorage.list.steer_tau);
	Serial.println(ParameterStorage.list.steer_rmax);
	Serial.println(ParameterStorage.list.steer_imax);
	Serial.println(ParameterStorage.list.YAW2SRV_SLP);
	Serial.println(ParameterStorage.list.YAW2SRV_RLL);
	Serial.println(ParameterStorage.list.max_yaw_aux);
	Serial.println(ParameterStorage.list.min_yaw_aux);


	Serial.println(ParameterStorage.list.L1_damping);
	Serial.println(ParameterStorage.list.L1_period);
	Serial.println(ParameterStorage.list.L1_bank);
	Serial.println(ParameterStorage.list.L1_gravity);

	Serial.println(ParameterStorage.list.max_thr_aux);
	Serial.println(ParameterStorage.list.min_thr_aux);
	Serial.println(ParameterStorage.list.max_thr_perc);
	Serial.println(ParameterStorage.list.min_thr_perc);
	Serial.println(ParameterStorage.list.cru_thr);	   


	Serial.println(ParameterStorage.list.slew_rate);
	Serial.println(ParameterStorage.list.max_speed);
	Serial.println(ParameterStorage.list.min_speed);
	Serial.println(ParameterStorage.list.cru_speed);
	Serial.println(ParameterStorage.list.land_speed);
	Serial.println(ParameterStorage.list.cru_altitude);
	Serial.println(ParameterStorage.list.APPROACH);

	Serial.println(ParameterStorage.list.TECS_thr_land);
	Serial.println(ParameterStorage.list.TECS_thr_tomax);
	Serial.println(ParameterStorage.list.AUTO_pitchmax);
	Serial.println(ParameterStorage.list.AUTO_pitchmin);
	Serial.println(ParameterStorage.list.TECS_land_pitchmax);
	Serial.println(ParameterStorage.list.TECS_land_pitchmin);
	Serial.println(ParameterStorage.list.arspdEnabled);
	Serial.println(ParameterStorage.list.TECS_stallPrevent);

	Serial.println(ParameterStorage.list.PowerModule_Gain);

	Serial.println(ParameterStorage.list.Accel2_offsetX);
	Serial.println(ParameterStorage.list.Accel2_offsetY);
	Serial.println(ParameterStorage.list.Accel2_offsetZ);
	Serial.println(ParameterStorage.list.Accel2_lsbX);   
	Serial.println(ParameterStorage.list.Accel2_lsbY);
	Serial.println(ParameterStorage.list.Accel2_lsbZ);
}

void
AP_Storage::WriteAll(void)
{	
  if(PARAMETER_VERSION != (uint8_t)EEPROM.read(3999))
  {
	 
	 // EEPROM ERASE
	 for(int index = 0; index < (sizeof(_parameter_list_t) + 50); index++)
	   EEPROM.write(3000+index, 0);
   
     // UPDATE BUFFER
	 for(int8_t x = 0; x <GROUP_DATA_SIZE; x++)
	 {		 
		 _f_un float_union;
		 float_union.val = pgm_read_float(&GROUP_DATA[x].val);

		 uint8_t pos     = (uint8_t)pgm_read_word(&GROUP_DATA[x].pos);
		 uint8_t len     = (uint8_t)pgm_read_word(&GROUP_DATA[x].len);
		 
		 // char un[16];
		 // strcpy_P(un, (char *)pgm_read_word(&GROUP_DATA[x].id));
		 // Serial.print(pgm_read_word(&(GROUP_DATA[x].id + 2)));
		 // Serial.print("		");
		 // Serial.println(un);
		 switch((uint8_t)pgm_read_word(&GROUP_DATA[x].type))
		 {
			 case FLOAT: {
					 _f_un float_union;
					 float value     = pgm_read_float(&GROUP_DATA[x].val);
					 float_union.val = value;
					 for(int i = pos; i< (pos + len); i++){
						ParameterStorage.paramBuffer[i] = float_union.buffer[i - pos];
					 }
				 }
				 break;
			 
			 case U8:{
					 _u8_un u8_union;
					 uint8_t value     = (uint8_t)pgm_read_float(&GROUP_DATA[x].val);
					 u8_union.val = value;
					 for(int i = pos; i< (pos + len); i++){
						ParameterStorage.paramBuffer[i] = u8_union.buffer[i - pos];
					 }
				 }
				 break;
			 
			 case I8:{
					 _i8_un i8_union;
					 int8_t value     = (int8_t)pgm_read_float(&GROUP_DATA[x].val);
					 i8_union.val = value;
					 for(int i = pos; i< (pos + len); i++){
						ParameterStorage.paramBuffer[i] = i8_union.buffer[i - pos];
					 }
				 }
				 break;
			 
			 case U16:{
					 _u16_un u16_union;
					 uint16_t value     = (uint16_t)pgm_read_float(&GROUP_DATA[x].val);
					 u16_union.val = value;
					 for(int i = pos; i< (pos + len); i++){
						ParameterStorage.paramBuffer[i] = u16_union.buffer[i - pos];
					 }
				 }
				 break;
			 
			 case I16:{
					 _i16_un i16_union;
					 int16_t value     = (int16_t)pgm_read_float(&GROUP_DATA[x].val);
					 i16_union.val = value;
					 for(int i = pos; i< (pos + len); i++){
						ParameterStorage.paramBuffer[i] = i16_union.buffer[i - pos];
					 }
				 }
				 break;
		 }
	  }
	  
	// UPDATE EEPROM
	for(int index = 0; index < sizeof(_parameter_list_t); index++)
		EEPROM.write(3000+index,ParameterStorage.paramBuffer[index]);
	   
	// PRINT VALUES
	#if PRINTING_ALL
	print_all();
	#endif
	   
	EEPROM.write(3999,PARAMETER_VERSION);	  
	Serial.println(F("AP_Parameters::Status	Parameters	Written"));
  }
  else
  {
	  Serial.println(F("AP_Parameters::Status	Parameters	UptoDate"));
  }
}


void
AP_Storage::initialise(HardwareSerial *_Port)
{    
  Port = _Port;
  
  // Code once
  WriteAll();
    
  // Read Parameters
  ReadAll();
  
  Serial.println(F("AP_Parameters::status	Initialised"));
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
   GROUP_DATA_SIZE, 
   param_index
  );  
  
  uint16_t len = mavlink_msg_to_send_buffer(buf,(msg)); 
 
  Port->write(buf,len);
}


// Gets parameter from paramBuffer
float 
AP_Storage::get_param(const uint8_t x){
	
	if(x >= GROUP_DATA_SIZE){
		Serial.println(F("Index	Outof	Range"));
		return 0;
	}
	
	 uint8_t type    = (uint8_t)pgm_read_word(&GROUP_DATA[x].type);
	 uint8_t pos     = (uint8_t)pgm_read_word(&GROUP_DATA[x].pos);
	 uint8_t len     = (uint8_t)pgm_read_word(&GROUP_DATA[x].len);
	 float val       = 0;
 
	 switch(type)
	 {
		 case FLOAT: {
				_f_un float_union;
				 for(int i = pos; i< (pos + len); i++){
					float_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = float_union.val;
				 }
			 }
			 return val;
		 
		 case U8:{
				 _u8_un u8_union;
				 for(int i = pos; i< (pos + len); i++){
					u8_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = u8_union.val;
				 }
			 }
			 return val;
		 
		 case I8:{
				 _i8_un i8_union;
				 for(int i = pos; i< (pos + len); i++){
					i8_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = i8_union.val;
				 }
			 }
			 return val;
		 
		 case U16:{
				 _u16_un u16_union;
				 for(int i = pos; i< (pos + len); i++){
					u16_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = u16_union.val;
				 }
			 }
			 return val;
		 
		 case I16:{
				 _i16_un i16_union;
				 for(int i = pos; i< (pos + len); i++){
					i16_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = i16_union.val;
				 }
			 }
			 return val;
	 }	
}


// Gets parameter from paramBuffer
float 
AP_Storage::get_param(const char param_id[16]){
	
	int8_t x  = -1;
	char ppm_id[16];
	for(int i = 0; i < GROUP_DATA_SIZE; i++){
		strcpy_P(ppm_id, (char *)pgm_read_word(&GROUP_DATA[i].id));
		if(strcmp(param_id, ppm_id) == 0)
		{
			x = i;
			break;
		}
	}
	
	if(x < 0)
	{
		Serial.println(F("Parameter	Not	Found"));
		return 0;
	}
	else if(x > GROUP_DATA_SIZE){
		Serial.println(F("Index	Outof	Range"));
		return 0;	
	}
	
	return get_param(x);
	
	/*
	uint8_t type  = (uint8_t)pgm_read_word(&GROUP_DATA[x].type);
	uint8_t pos   = (uint8_t)pgm_read_word(&GROUP_DATA[x].pos);
	uint8_t len   = (uint8_t)pgm_read_word(&GROUP_DATA[x].len);
	float val     = 0;
 
	switch(type)
	{
		 case FLOAT: {
				_f_un float_union;
				 for(int i = pos; i< (pos + len); i++){
					float_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = float_union.val;
				 }
			 }
			 return val;
		 
		 case U8:{
				 _u8_un u8_union;
				 for(int i = pos; i< (pos + len); i++){
					u8_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = u8_union.val;
				 }
			 }
			 return val;
		 
		 case I8:{
				 _i8_un i8_union;
				 for(int i = pos; i< (pos + len); i++){
					i8_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = i8_union.val;
				 }
			 }
			 return val;
		 
		 case U16:{
				 _u16_un u16_union;
				 for(int i = pos; i< (pos + len); i++){
					u16_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = u16_union.val;
				 }
			 }
			 return val;
		 
		 case I16:{
				 _i16_un i16_union;
				 for(int i = pos; i< (pos + len); i++){
					i16_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 val = i16_union.val;
				 }
			 }
			 return val;
	 }
*/	 
}



// Get Param Type given index
uint8_t 
AP_Storage::get_type(const uint8_t x){
	
	if(x >= GROUP_DATA_SIZE)
		return 0;
	
	uint8_t type = (uint8_t)pgm_read_word(&GROUP_DATA[x].type);

	return type;
}


// Get parameter type given parameterId
uint8_t 
AP_Storage::get_type(const char parameterId[16]){

	char ppm_id[16];
	for(int i = 0; i < GROUP_DATA_SIZE; i++)
	{
		strcpy_P(ppm_id, (char *)pgm_read_word(&GROUP_DATA[i].id));
		if(strcmp(parameterId, ppm_id) == 0)	
			return (uint8_t)pgm_read_word(&GROUP_DATA[i].type);
	}
}


// Retrun the Index of this ParameterId
uint8_t AP_Storage::find_parameter(const char param_id[16]){

  uint8_t parameter_index = 0;
  char param_buffer[16];
  while(parameter_index < GROUP_DATA_SIZE)
  {
    strcpy_P(param_buffer, (char *)pgm_read_word(&GROUP_DATA[parameter_index].id)); // copy parameter id from flash into parameter_id variable
    if(strcmp(param_id, param_buffer) == 0)
		return parameter_index;
	else
		parameter_index++;
  }
}



void 
AP_Storage::SendParamList(byte UAV_ID, mavlink_message_t *msg)
{	
  union _f_un param_un;                                                            // only one copy is created instead of millions in the while loop..
  char  parameter_id[16];
  byte  parameter_index = 0;
  
  while(parameter_index < GROUP_DATA_SIZE)                             // this is the number of parameters
  {      
    strcpy_P(parameter_id, (char *)pgm_read_word(&GROUP_DATA[parameter_index].id)); // copy parameter id from flash into parameter_id variable
     
	mavlink_sendParameter(UAV_ID, parameter_id, get_param(parameter_index), (uint8_t)pgm_read_word(&GROUP_DATA[parameter_index].type), parameter_index,  msg);  
    
	parameter_index++;    
    
	delay(100);  // 10Hz
  }  
  parameter_index = 0;   
}


void AP_Storage::UpdateStorage(const byte UAV_ID, const char param_id[16], float param_value, mavlink_message_t *msg)
{  
  uint8_t parameter_index = -1;
  
  parameter_index = find_parameter(param_id);
  
  if(parameter_index < 0)
	  return;
  else{
	uint8_t type = (uint8_t)pgm_read_word(&GROUP_DATA[parameter_index].type);
	uint8_t pos  = (uint8_t)pgm_read_word(&GROUP_DATA[parameter_index].pos);
	uint8_t len  = (uint8_t)pgm_read_word(&GROUP_DATA[parameter_index].len);	
	
	// EEPROM UPDATE
	switch(type)
	{
		case FLOAT: {
				_f_un float_union;
				float_union.val = param_value;
				 for(int i = pos; i< (pos + len); i++){
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, float_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = float_union.buffer[i - pos];
				 }
			 }
			 break;;
		 
		case U8:{
				 _u8_un u8_union;
				u8_union.val = param_value;
				
				 for(int i = pos; i< (pos + len); i++){
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, u8_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = u8_union.buffer[i - pos];
				 }
			 }
			 break;
		 
		case I8:{
				 _i8_un i8_union;
				 i8_union.val = param_value;
				 for(int i = pos; i< (pos + len); i++){
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, i8_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = i8_union.buffer[i - pos];
				 }
			 }
			 break;
		 
		case U16:{
				 _u16_un u16_union;
				 u16_union.val = param_value;
				 for(int i = pos; i< (pos + len); i++){
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, u16_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = u16_union.buffer[i - pos];
				 }
			 }
			 break;
		 
		case I16:{
				 _i16_un i16_union;
				 i16_union.val = param_value;
				 for(int i = pos; i< (pos + len); i++){
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, i16_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = i16_union.buffer[i - pos];
				 }
			 }
			 break;
   }	
	
   // SEND PARAMETER
   float value = get_param(parameter_index);
   mavlink_sendParameter(UAV_ID, param_id, value, type, parameter_index, msg); 
  
  }
}
  

