#define CHARS_TABLE  1
#include "AP_Parameters.h"

AP_Storage  AP_params;

AP_Storage::AP_Storage()
{ 
};


// Gets Data From EEPROM
void
AP_Storage::ReadAll(void)
{
   for(int x = 0; x < (sizeof(_parameter_list_t)); x++ )
   {
      ParameterStorage.paramBuffer[x] = EEPROM.read(3000+x);
   }
   
   Serial.println(F("AP_Parameters::status	Read	All	Parameters"));
}

void 
AP_Storage::print_all(){
	for(int i = 0; i< GROUP_DATA_SIZE; i++)
		Serial.println(get_param(i).val);
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


// Gets parameter from paramBuffer
_Param_t 
AP_Storage::get_param(const uint16_t x){
	_Param_t	pm;
	
	if(x >= GROUP_DATA_SIZE){
		Serial.println(F("AP_params::Index	Outof	Range"));
		return pm;
	}
	
	 strcpy_P(pm.id, (char *)pgm_read_word(&GROUP_DATA[x].id));
	 uint8_t type    = (uint8_t)pgm_read_word(&GROUP_DATA[x].type);
	 uint8_t pos     = (uint8_t)pgm_read_word(&GROUP_DATA[x].pos);
	 uint8_t len     = (uint8_t)pgm_read_word(&GROUP_DATA[x].len);
	 float val       = 0;
	 pm.index 		 = x;
	 
	 switch(type)
	 {
		 case FLOAT: {
				_f_un float_union;
				 for(int i = pos; i< (pos + len); i++){
					float_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 }
				val = float_union.val;
				pm.val   = val;
				pm.type  = type;
				pm.count = GROUP_DATA_SIZE;
			 }
			 return pm;
		 
		 case U8:{
				 _u8_un u8_union;
				 for(int i = pos; i< (pos + len); i++){
					u8_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 }
				val = u8_union.val;
				pm.val   = val;
				pm.type  = type;
				pm.count = GROUP_DATA_SIZE;
			 }
			 return pm;
		 
		 case I8:{
				 _i8_un i8_union;
				 for(int i = pos; i< (pos + len); i++){
					i8_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 }
				val = i8_union.val;
				pm.val   = val;
				pm.type  = type;
				pm.count = GROUP_DATA_SIZE;
			 }
			 return pm;
		 
		 case U16:{
				 _u16_un u16_union;
				 for(int i = pos; i< (pos + len); i++){
					u16_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 }
				val = u16_union.val;
				pm.val   = val;
				pm.type  = type;
				pm.count = GROUP_DATA_SIZE;
			 }
			 return pm;
		 
		 case I16:{
				 _i16_un i16_union;
				 for(int i = pos; i< (pos + len); i++){
					i16_union.buffer[i - pos] = ParameterStorage.paramBuffer[i];
				 }
			    val = i16_union.val;
				pm.val   = val;
				pm.type  = type;
				pm.count = GROUP_DATA_SIZE;
			 }
			 return pm;
	 }	
}


// Gets parameter from paramBuffer
_Param_t 
AP_Storage::get_param(const char param_id[16]){
	
	int16_t x  = -1;
	char ppm_id[16];
	for(int i = 0; i < GROUP_DATA_SIZE; i++){
		strcpy_P(ppm_id, (char *)pgm_read_word(&GROUP_DATA[i].id));
		if(strcmp(param_id, ppm_id) == 0)
		{
			x = i;
			break;
		}
	}
	
	return get_param(x); 
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
uint16_t AP_Storage::get_index(const char param_id[16]){

  uint16_t parameter_index = 0;
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


void AP_Storage::UpdateStorage(const char param_id[16], float param_value, bool write_eeprom)
{  
  int16_t parameter_index = -1;
  
  parameter_index 		 = get_index(param_id);
  
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
					if(write_eeprom)
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, float_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = float_union.buffer[i - pos];
				 }
			 }
			 break;;
		 
		case U8:{
				 _u8_un u8_union;
				u8_union.val = param_value;
				
				 for(int i = pos; i< (pos + len); i++){
					if(write_eeprom)
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, u8_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = u8_union.buffer[i - pos];
				 }
			 }
			 break;
		 
		case I8:{
				 _i8_un i8_union;
				 i8_union.val = param_value;
				 for(int i = pos; i< (pos + len); i++){
					if(write_eeprom)
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, i8_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = i8_union.buffer[i - pos];
				 }
			 }
			 break;
		 
		case U16:{
				 _u16_un u16_union;
				 u16_union.val = param_value;
				 for(int i = pos; i< (pos + len); i++){
					if(write_eeprom)
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, u16_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = u16_union.buffer[i - pos];
				 }
			 }
			 break;
		 
		case I16:{
				 _i16_un i16_union;
				 i16_union.val = param_value;
				 for(int i = pos; i< (pos + len); i++){
					if(write_eeprom)
					EEPROM.write(EEPROM_PARAMETER_PAGE + i, i16_union.buffer[i - pos]);
					ParameterStorage.paramBuffer[i] = i16_union.buffer[i - pos];
				 }
			 }
			 break;
   }	
  }
}
  

