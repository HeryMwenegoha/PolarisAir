/*@ Library implemented by Hery A Mwenegoha (C) 15/07/2019
 */
#pragma once

#include "Arduino.h"
#include "Wire.h"

#define BMP388_ADDRESS 	0x77
#define PRODUCT_ID     	0x50

#define WHO_AM_I       	0x00 //
#define ERR_REG     	0x02 
#define STATUS_REG  	0x03 // Bit5-Press, Bit6-Temp

#define PRES_REG1   	0x04 // XLSB    u24
#define PRES_REG2   	0x05 // LSB
#define PRES_REG3   	0x06 // MSB

#define TEMP_REG1   	0x07  // XLSB 7-0 u24
#define TEMP_REG2   	0x08  // LSB  15-8
#define TEMP_REG3   	0x09  // MSB  23-16

#define TIME_REG1   	0x0C
#define TIME_REG2   	0x0D
#define TIME_REG3   	0x0E

#define PWR_REG     	0x1B
#define OSR_REG     	0x1C
#define ODR_REG     	0x1D
#define CONFIG_REG  	0x1F 
#define CMD_REG     	0x78 

#define CALIB_REG1  	0x31
#define CALIB_REGN  	0x57


class AP_BMP388
{
  public:
  AP_BMP388();
  bool   begin();
  
  struct  BMP388_calib_data{
  float   par_p11;
  float   par_p10;
  float   par_p9;
  float   par_p8;
  float   par_p7;
  float   par_p6;
  float   par_p5;
  float   par_p4;
  float   par_p3;
  float   par_p2;
  float   par_p1;
  float   par_t3;
  float   par_t2;
  float   par_t1;

  float  t_lin; // raw compensated temperature [K]
  float  p_lin; // raw compensated pressure    [Pa]
  };
  struct BMP388_calib_data calib_data;
  
  void   read();
  
  float  BMP388_compensate_temperature(uint32_t uncomp_temp, struct BMP388_calib_data *calib_datA);
  
  float  BMP388_compensate_pressure(uint32_t uncomp_press, struct BMP388_calib_data *calib_data);

  uint32_t _last_update_msec;
  
  bool _have_sens;
  
  float baseline;
  
  bool     temperature(float &temp);
  bool     pressure(float  &press);
  float    baseline_pressure();
  uint32_t timeStamp_msec();
};
