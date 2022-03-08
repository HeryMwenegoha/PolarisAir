#include "BMP388.h"


AP_BMP388::AP_BMP388()
{
  _have_sens    = false;
}


bool AP_BMP388::begin()
{
	// put your setup code here, to run once:
	if(PRODUCT_ID != Wire.read8(BMP388_ADDRESS, WHO_AM_I))
	{
		_have_sens = false;
		return false;
	}
	else
	{
  
		// Soft reset - Device goes into sleep mode
		Wire.write8(BMP388_ADDRESS, CMD_REG, 0xB6);
		delay(15);

		// IIR filter setup : coef_3-[0x02<<1], coef_7-[0x03<<1]
		Wire.write8(BMP388_ADDRESS, CONFIG_REG, (0x02 << 1));
		delay(15);

		// Set ODR : prescaler 1 -> 25Hz
		Wire.write8(BMP388_ADDRESS, ODR_REG, 0x03);
		delay(15);

		// Set OSR : x16-Pressure [0x04], x32-[0x05] and x2-Temp [0x01 << 3] 
		Wire.write8(BMP388_ADDRESS, OSR_REG, ((0x04) | (0x01 << 3)));
		delay(15);

		// Enable Pressure and Temperature and Put in Normal Mode
		Wire.write8(BMP388_ADDRESS, PWR_REG, ((0x03) | (0x03 << 4)));
		delay(200);
		

		// Read Data into Memory
		uint16_t NVM_PAR_T1 = (uint16_t)(Wire.read8(BMP388_ADDRESS, 0x32) << 8 | Wire.read8(BMP388_ADDRESS, 0x31)); //uint16_t NVM_PAR_T1 = (uint16_t)(T1_msb << 8 | T1_lsb);

		uint16_t NVM_PAR_T2 = (uint16_t)(Wire.read8(BMP388_ADDRESS, 0x34) << 8 | Wire.read8(BMP388_ADDRESS, 0x33)); //uint16_t NVM_PAR_T2 = (uint16_t)(T2_msb << 8 | T2_lsb);

		int8_t NVM_PAR_T3 	= (int8_t)Wire.read8(BMP388_ADDRESS, 0x35);

		int16_t NVM_PAR_P1 	= (int16_t)(Wire.read8(BMP388_ADDRESS, 0x37) << 8 | Wire.read8(BMP388_ADDRESS, 0x36)); //int16_t NVM_PAR_P1 = (int16_t)(P1_msb << 8 | P1_lsb);

		int16_t NVM_PAR_P2 	= (int16_t)(Wire.read8(BMP388_ADDRESS, 0x39) << 8 | Wire.read8(BMP388_ADDRESS, 0x38)); //int16_t NVM_PAR_P2 = (int16_t)(P2_msb << 8 | P2_lsb);

		int8_t NVM_PAR_P3 	= (int8_t)Wire.read8(BMP388_ADDRESS, 0x3A);

		int8_t NVM_PAR_P4 	= (int8_t)Wire.read8(BMP388_ADDRESS, 0x3B);

		uint16_t NVM_PAR_P5 = (uint16_t)(Wire.read8(BMP388_ADDRESS, 0x3D) << 8 | Wire.read8(BMP388_ADDRESS, 0x3C)); //uint16_t NVM_PAR_P5 = (uint16_t)(P5_msb << 8 | P5_lsb);

		uint16_t NVM_PAR_P6 = (uint16_t)(Wire.read8(BMP388_ADDRESS, 0x3F) << 8 | Wire.read8(BMP388_ADDRESS, 0x3E)); //uint16_t NVM_PAR_P6 = (uint16_t)(P6_msb << 8 | P6_lsb);

		int8_t NVM_PAR_P7 	= (int8_t)Wire.read8(BMP388_ADDRESS, 0x40);

		int8_t NVM_PAR_P8 	= (int8_t)Wire.read8(BMP388_ADDRESS, 0x41); 

		int16_t NVM_PAR_P9 	= (int16_t)(Wire.read8(BMP388_ADDRESS, 0x43) << 8 |  Wire.read8(BMP388_ADDRESS, 0x42));//int16_t NVM_PAR_P9 = (int16_t)(P9_msb << 8 | P9_lsb);

		int8_t  NVM_PAR_P10 = (int8_t)Wire.read8(BMP388_ADDRESS, 0x44); 

		int8_t  NVM_PAR_P11 = (int8_t)Wire.read8(BMP388_ADDRESS, 0x45);

		calib_data.par_p11=(float)(NVM_PAR_P11/pow(2,65));
		calib_data.par_p10=(float)(NVM_PAR_P10/pow(2,48));
		calib_data.par_p9=(float)(NVM_PAR_P9/pow(2,48));
		calib_data.par_p8=(float)(NVM_PAR_P8/pow(2,15));
		calib_data.par_p7=(float)(NVM_PAR_P7/pow(2,8));
		calib_data.par_p6=(float)(NVM_PAR_P6/pow(2,6));
		calib_data.par_p5=(float)(NVM_PAR_P5/pow(2,-3));
		calib_data.par_p4=(float)(NVM_PAR_P4/pow(2,37));
		calib_data.par_p3=(float)(NVM_PAR_P3/pow(2,32));
		calib_data.par_p2=(float)((NVM_PAR_P2-pow(2,14))/pow(2,29));
		calib_data.par_p1=(float)((NVM_PAR_P1-pow(2,14))/pow(2,20));
		calib_data.par_t3=(float)(NVM_PAR_T3/pow(2,48));
		calib_data.par_t2=(float)(NVM_PAR_T2/pow(2,30));
		calib_data.par_t1=(float)(NVM_PAR_T1/pow(2,-8));


		_last_update_msec = millis();
		_have_sens        = true; // read will not work otherwise
		delay(40);				  // allow some time to pass 40ms -> 25Hz
		
		int idx = 25; // 1sec
		float _pressure;
		float _temperature;
		while(idx-- > 0)
		{
		   read();
		   
		   if(idx >= 15){
			   _pressure     = calib_data.p_lin;
			   _temperature  = calib_data.t_lin;;		   
		   }else{
			  _temperature = 0.95 * calib_data.t_lin  + 0.05 * _temperature; 
			  _pressure    = 0.95 * calib_data.p_lin  + 0.05 * _pressure; 
		   }
		   
		   delay(40);
		}
		
		baseline  	   = _pressure;

		Serial.print(F("BMP388 Found : "));
		Serial.print(F("\t"));
		Serial.print(_temperature);
		Serial.print(F("\t"));
		Serial.println(baseline);
		
		
		return true;
	}	
}


// read Sensor
void AP_BMP388::read()
{
  if(!_have_sens)	return;
	
  if((millis()-_last_update_msec) >= 40)
  {
    // update Time
    _last_update_msec = millis();
    
    // put your main code here, to run repeatedly:
    Wire.beginTransmission(BMP388_ADDRESS);
    Wire.write((STATUS_REG));
    Wire.endTransmission();
  
    Wire.requestFrom(BMP388_ADDRESS, 7);
    uint32_t _start_msec = millis();
    while(Wire.available() < 7){
        if((millis() - _start_msec) >= 50){
          Serial.println(F("BMP388 I2C Timeout"));
          return;
        }
    }
      
    const uint8_t _status  =   Wire.read();
    const uint8_t _statusP = ((_status & 0x20) >> 5);
    const uint8_t _statusT = ((_status & 0x40) >> 6);
    uint8_t buffer[9];
        
    if((_statusP == 1) && (_statusT == 1)){
        buffer[0] = Wire.read();
        buffer[1] = Wire.read();        
        buffer[2] = Wire.read();
        buffer[3] = Wire.read();  
        buffer[4] = Wire.read();
        buffer[5] = Wire.read();  

        uint32_t P_xlsb = (uint32_t)buffer[0];       // 20bit resolution
        uint32_t P_lsb  = (uint32_t)buffer[1] << 8;
        uint32_t P_msb  = (uint32_t)buffer[2] << 16;

        uint32_t uncomp_Press = P_msb | P_lsb | P_xlsb; 

        uint32_t T_xlsb = (uint32_t) buffer[3];     // 17bit resolution
        uint32_t T_lsb  = (uint32_t) buffer[4] << 8;
        uint32_t T_msb  = (uint32_t) buffer[5] << 16;

        uint32_t uncomp_Temp = T_msb  | T_lsb  | T_xlsb; 

        BMP388_compensate_temperature(uncomp_Temp, &calib_data);
        BMP388_compensate_pressure(uncomp_Press, &calib_data);
    }
  }	
}


// compensate Temperature : code by Bosch
float AP_BMP388::BMP388_compensate_temperature(uint32_t uncomp_temp, struct BMP388_calib_data *calib_datA)
{
  float partial_data1;
  float partial_data2;
  partial_data1 = (float)(uncomp_temp - calib_datA->par_t1);
  partial_data2 = (float)(partial_data1 * calib_datA->par_t2);
  calib_datA->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_datA->par_t3;
  return calib_datA->t_lin;
}


// compensate Pressure : code by Bosch
 float AP_BMP388::BMP388_compensate_pressure(uint32_t uncomp_press, struct BMP388_calib_data *calib_data)
 {
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;
	partial_data1 = calib_data->par_p6 * calib_data->t_lin;
	partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calib_data->par_p2 * calib_data->t_lin;
	partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out2 = (float)uncomp_press *
	(calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
	calib_data->p_lin = partial_out1 + partial_out2 + partial_data4;
	return calib_data->p_lin;
}


// Get Temperature [K]
bool AP_BMP388::temperature(float &temp)
{
	if((millis() - _last_update_msec > 250))
		return false;
	temp = calib_data.t_lin;
	return true;
}
  

// Get Pressure [Pa]
bool AP_BMP388::pressure(float  &press)
{
	if((millis() - _last_update_msec > 250))
		return false;
	press = calib_data.p_lin;
	return true;
}

// Get baseline pressure
float AP_BMP388::baseline_pressure()
{
	return baseline;
}


// Get TimeStamp : Since Start - Time Signal was Acquired
uint32_t AP_BMP388::timeStamp_msec()
{
	return _last_update_msec;
}