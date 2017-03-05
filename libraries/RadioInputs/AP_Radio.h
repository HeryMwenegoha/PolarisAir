#pragma once
#include "Arduino.h"

#define MYPIN1 A8
#define MYPIN2 A9
#define MYPIN3 A10
#define MYPIN4 A11
#define MYPIN5 A12
#define MYPIN6 A13
#define MYPIN7 A14
#define MYPIN8 A15

class AP_Radio
{
	public:
	AP_Radio();
	static void rc_input_setup();
	static byte number_declarations;
	
	uint16_t chan1(); // goes to roll servo
	uint16_t chan2(); // goes to pitch servo
	uint16_t chan3(); // goes to throttle servo
	uint16_t chan4(); // goes to yaw servo
	uint16_t chan5(); // goes to aux1
	uint16_t chan6(); // goes to aux2 
	uint16_t chan7(); // goes to aux3
	uint16_t chan8(); // goes to aux4
	
	private:
	static volatile  uint16_t  delta_roll;
	static volatile  uint16_t  delta_pitch;
	static volatile  uint16_t delta_throttle;  
	static volatile  uint16_t delta_yaw;
	static volatile  uint16_t delta_aux1;
	static volatile  uint16_t delta_aux2;
	static volatile  uint16_t delta_aux3;
	static volatile  uint16_t delta_aux4;

	static volatile  uint16_t time_H;
	static volatile  uint16_t time_L;
	static volatile  uint16_t pitch_time_H;
	static volatile  uint16_t pitch_time_L;
	static volatile  uint16_t throttle_time_H;
	static volatile  uint16_t throttle_time_L;
	static volatile  uint16_t yaw_time_H;
	static volatile  uint16_t yaw_time_L;
	static volatile  uint16_t aux1_time_H;
	static volatile  uint16_t aux1_time_L;
	static volatile  uint16_t aux2_time_H;
	static volatile  uint16_t aux2_time_L;
	static volatile  uint16_t aux3_time_H;
	static volatile  uint16_t aux3_time_L;
	static volatile  uint16_t aux4_time_H;
	static volatile  uint16_t aux4_time_L;
	
	static void pin1func();
	static void pin2func();
	static void pin3func();
	static void pin4func();
	static void pin5func();
	static void pin6func();
	static void pin7func();
	static void pin8func();
	 
};
