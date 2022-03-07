/*@ Library implemented by Hery A Mwenegoha (C) 20/06/2015
 */
#pragma once

#include "Arduino.h"
#include "Wire.h"

#define BMP180_ADDR 				0x77 // 7-bit address
#define	BMP180_REG_CONTROL 			0xF4
#define	BMP180_REG_RESULT 			0xF6
#define	BMP180_COMMAND_TEMPERATURE	0x2E
#define	BMP180_COMMAND_PRESSURE0 	0x34
#define	BMP180_COMMAND_PRESSURE1 	0x74
#define	BMP180_COMMAND_PRESSURE2 	0xB4
#define	BMP180_COMMAND_PRESSURE3 	0xF4

class SFE_BMP180
{
  public:
  SFE_BMP180();
  bool   begin();
  char   startTemperature(void);
  char   getTemperature(double &T);
  //char getTemperature(double &T);
  char   startPressure(char oversampling);
  char   getPressure(double &P, double &T);
  double sealevel(double P, double A);
  void   get_altitude(float &Pressure);
  float  get_altitude(void);
  
  
  float  altitude;
  char   getError(void);
  float  getBaselinePressure();
  bool   read();
  bool   perform_measurement_NoTimer(void);
  float  baseline;  
  
  bool 	 temperature(float &);
  bool 	 pressure(float &);
  
  float  baseline_pressure();
  uint32_t last_update_msec;
  
  private:
  bool 	   data_ready();
  char 	   readInt(char address, int16_t &value);
  char 	   readUInt(char address, uint16_t &value);
  char 	   readBytes(unsigned char *values, char length);
  char 	   writeBytes(unsigned char *values, char length); 
  int16_t  AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
  uint16_t AC4,AC5,AC6; 
  double   c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2, _raw_pressure;
  double  _temperature;
  char 	   _error;
  byte     STATE_MACHINE;
  byte     _state;
  byte     _update_count;
  bool	   _base_update;
  uint32_t _last_update_Msec;
  uint32_t _last_cmd_temp_read_msec;
  uint32_t _last_cmd_pres_read_msec;
  
  double _pressure_sum; double _avg_pressure;
  byte   _count;
};
