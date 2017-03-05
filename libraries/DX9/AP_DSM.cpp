#include "AP_DSM.h"
#define DX_FRAME_CHANNELS 7

//volatile uint8_t AP_DSM::_dsm_buffer[16] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500, 1500, 1500,1500,1500};
void AP_DSM::initialise(HardwareSerial *_Port)
{
  Port = _Port;
  
  Port->begin(115200);
  
  delay(50);
  
  for(int i = 0; i < 9; i++){
	  values[i] = 1500;
  }
  
  _last_dsm_usec = micros();
  _update_usec   = micros();
  
  DT = 0;
}

// currently disabled in the main decode programe and the dsm shift has been hard coded to 11
void AP_DSM::guess_format(bool reset_variables, const uint8_t dsm_buffer[16])
{	
	if(reset_variables){
		cs10 = 0;
		cs11 = 0;
		samples = 0;
		_dsm_shift = 0;
		return;
	}
	
	for(int i = 0; i < DX_FRAME_CHANNELS; i++){
		const uint8_t *dp = &dsm_buffer[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		uint16_t channel, value;
		
		// check if we can decode in 10 bit shift.
		if(decode_channel(raw, 10, &channel, &value) && (channel < 31)){
			cs10 = cs10 | (1 << channel);
		}
		
		// check if we can decode in 11 bit shift.
		if(decode_channel(raw, 11, &channel, &value) && (channel < 31)){
			cs11 = cs11 | (1 << channel);
		}
	}
	
	if(samples++ < 5){
		return;
	}
	
	uint32_t masks[] = {
		0x3f,	/* 6 channels (DX6) */
		0x7f,	/* 7 channels (DX7) */
		0xff,	/* 8 channels (DX8) */
		0x1ff,	/* 9 channels (DX9, etc.) */
		0x3ff,	/* 10 channels (DX10) */
		0x1fff,	/* 13 channels (DX10t) */
		0x3fff	/* 18 channels (DX10) */
	};
	
	unsigned votes10 = 0;
	unsigned votes11 = 0;

	for(int i = 0; i < sizeof(masks); i++){
		
		if(cs10 == masks[i]){
			votes10++;
		}
		
		if(cs11 == masks[i]){
			votes11++;
		}
	}
	
	if((votes11 == 1) && (votes10 == 0)){
		_dsm_shift = 11;
		Serial.println("DCM Shift 11");
		return;
	}
	
	if((votes10 == 1) && (votes11 == 0)){
		_dsm_shift = 10;
		Serial.println("DCM Shift 10");
		return;
	}
	
	Serial.println("Format Guess Failed");
	//_dsm_shift = 11;
	guess_format(true, dsm_buffer);
}



boolean AP_DSM::decode_channel(uint16_t _raw, uint16_t shift,  uint16_t *channel, uint16_t *value)
{
  // decode channel and value
  if(_raw == 0xffff)
    return false;
  
  *channel    = (_raw >> shift) & 0xf;
  uint16_t data_mask = (1 << shift) - 1;
  *value     = _raw & data_mask;

  return true;
}

// New
static uint8_t dsm_buffer[32];
static uint8_t _dsm_buffer[24];
static uint16_t frame_counter   = 0;
static uint64_t _last_rx_usec   = 0;
static uint32_t dsm_frame_drops = 0;
void AP_DSM::read_stream(){
	
	uint64_t now = micros();
	bool ret = false;
	
	// Read Stream
	int i = 0;
	if(Port->available()){
		while(i < 32 && Port->available()){
			dsm_buffer[i] = Port->read();
			i++;
			if(i >= 32)
				break;
		}
		ret = true;
	}
	
	if(i < 1)
		return;
	/*
	Serial.print("i ");
	Serial.println(i);
	*/
	
	bool decode_ret   = false;
	bool decode_state = false;
	int len		      = i;
	for(int d = 0; d < len; d++){
		if(frame_counter == 24){
			frame_counter = 0;
			decode_state  = false;
		}
		
		if(frame_counter == 16){
			frame_counter = 0;
			decode_state  = false;
		}
		
		switch(decode_state){
			case false:
			if((now - _last_rx_usec) > 5000){
				decode_state = true;
				frame_counter  = 0;
				_dsm_buffer[frame_counter++] = dsm_buffer[d];
				//Serial.println("NP");
			}			
			break;
			
			
			case true:{
				_dsm_buffer[frame_counter++] = dsm_buffer[d];
				
				if(frame_counter < 16){
					break;
				}				
				
				//Serial.println(frame_counter);
				
				decode_ret = decode_stream();			
				frame_counter = 0;
				
				// decoding failed
				if(decode_ret == false){
					decode_state = false;
					dsm_frame_drops++;
				}
			}
			break;
			
			default:
			decode_ret = false;
			break;

		}	
	}
	
	_last_rx_usec = now;

	
	/*
	uint8_t dsm_buffer[255];
	int16_t index 	= 0;
	
	while(Port->available()){
		dsm_buffer[index++] = Port->read();
	}
	
	if(index == 16){
		while(index > -1){
			index--;
			_dsm_buffer[index] = dsm_buffer[index];	
		}
	}
	*/
	
	
}

bool AP_DSM::decode_stream()
{ 
	  // if we havent received anything for the past 1 second we try and guess the format
	  uint64_t tnow = micros();
	  DT = static_cast<float>(tnow - _last_dsm_usec) * 1e-6f;
	  _last_dsm_usec = tnow;
	  if((DT > 1.0f) && (_dsm_shift != 0)){
		// guess_format(true, _dsm_buffer);
	  }
	  
	  // we dont know the dsm shift therefore update
	  if(_dsm_shift == 0){
		 //guess_format(false, _dsm_buffer);
	  }

      // i will work with 7 channels recalling that address 0 and 1 of dsm buffer are meant for fades count.    
      for(int i = 0; i < DX_FRAME_CHANNELS; i++){
        const uint8_t *_dsm = &_dsm_buffer[2 + (2*i)];
        uint16_t _raw = (_dsm[0] << 8) | _dsm[1];
        uint16_t channel, value;       

        // Skip the rest of the for loop if decoding has failed
        if(!decode_channel(_raw, 11 ,&channel, &value)){
          continue; 
        }

        // Skip the rest of the iteration if channel number is greater than 7 and go onto next iteration
        if(channel > 8){
          continue;
        }
		
		if(_dsm_shift == 10){
			//value *= 2;
		}

        // scale values 0/2048 to normal ppm us of 1000 to 2000
        value = ((((int16_t)value - 1024)* 1000)/1700) + 1500;

        // Re-assign certain spektrum channels which have been defined below
        /* Spketrum Channel Assignment
        Channel Identifiers
        ID Name
        0 Throttle
        1 Aileron
        2 Elevator
        3 Rudder
        4 Gear
        5 Aux 1
        6 Aux 2
        7 Aux 3
        8 Aux 4
        9 Aux 5
        10 Aux 6
        11 Aux 7 
        */
        
        switch(channel)
        {
          case 0:
            channel = 2;
            break;

          case 1:
            channel = 0;
            break;

          case 2:
            channel = 1;
            break;

          default:
            break;
        }

        values[channel] = value;
      }
	  
	  
	  for(int x = 0; x < 9; x++){
		  if(values[x] < 850 || values[x] > 2350){
			  Serial.println(F("DSMX Range Fail"));
			  return false;
		  }
	  }
	  
	  return true;
}
