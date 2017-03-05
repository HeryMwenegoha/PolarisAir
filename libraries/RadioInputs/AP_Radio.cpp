#include "AP_Radio.h"
#define  NO_PORTB_PINCHANGES
#include "PinChangeInt/PinChangeInt.h"

volatile  uint16_t AP_Radio::delta_roll	     = 1500;
volatile  uint16_t AP_Radio::delta_pitch	 = 1500;
volatile  uint16_t AP_Radio::delta_throttle  = 1500;  
volatile  uint16_t AP_Radio::delta_yaw       = 1500;
volatile  uint16_t AP_Radio::delta_aux1      = 1500;
volatile  uint16_t AP_Radio::delta_aux2 	 = 1500;
volatile  uint16_t AP_Radio::delta_aux3      = 1500;
volatile  uint16_t AP_Radio::delta_aux4      = 1500;

volatile  uint16_t AP_Radio::time_H		 = 1500;
volatile  uint16_t AP_Radio::time_L		 = 1500;
volatile uint16_t  AP_Radio::pitch_time_H	 = 1500;
volatile  uint16_t AP_Radio::pitch_time_L		 = 1500;
volatile  uint16_t AP_Radio::throttle_time_H	= 1500;
volatile  uint16_t AP_Radio::throttle_time_L	= 1500;
volatile  uint16_t AP_Radio::yaw_time_H		= 1500;
volatile  uint16_t AP_Radio::yaw_time_L	    = 1500;
volatile  uint16_t AP_Radio::aux1_time_H		= 1500;
volatile  uint16_t AP_Radio::aux1_time_L		= 1500;
volatile  uint16_t AP_Radio::aux2_time_H		= 1500;
volatile  uint16_t AP_Radio::aux2_time_L		= 1500;
volatile  uint16_t AP_Radio::aux3_time_H		= 1500;
volatile  uint16_t AP_Radio::aux3_time_L		= 1500;
volatile  uint16_t AP_Radio::aux4_time_H		= 1500;
volatile  uint16_t AP_Radio::aux4_time_L		= 1500;

byte AP_Radio::number_declarations = 0;

AP_Radio::AP_Radio()
{
	number_declarations++;
	pinMode(MYPIN1, INPUT_PULLUP);
	attachPinChangeInterrupt(MYPIN1, AP_Radio::pin1func, CHANGE);  
	pinMode(MYPIN2, INPUT_PULLUP);
	attachPinChangeInterrupt(MYPIN2, AP_Radio::pin2func, CHANGE);
	pinMode(MYPIN3, INPUT_PULLUP);
	attachPinChangeInterrupt(MYPIN3, AP_Radio::pin3func, CHANGE);
	pinMode(MYPIN4, INPUT_PULLUP);
	attachPinChangeInterrupt(MYPIN4, AP_Radio::pin4func, CHANGE);
	pinMode(MYPIN5, INPUT_PULLUP);
	attachPinChangeInterrupt(MYPIN5, AP_Radio::pin5func, CHANGE);
	pinMode(MYPIN6, INPUT_PULLUP);
	attachPinChangeInterrupt(MYPIN6, AP_Radio::pin6func, CHANGE);
	pinMode(MYPIN7, INPUT_PULLUP);
	attachPinChangeInterrupt(MYPIN7, AP_Radio::pin7func, CHANGE); 
	pinMode(MYPIN8, INPUT_PULLUP);
	attachPinChangeInterrupt(MYPIN8, AP_Radio::pin8func, CHANGE); 

	/*
	channel1 = 1500;
	channel2 = 1500;
	channel3 = 850;
	channel4 = 1500;
	channel5 = 1500;
	channel6 = 1500;
	channel7 = 1500;
	channel8 = 1500;  
	*/
}

void AP_Radio::rc_input_setup()
{
 	
}

uint16_t AP_Radio::chan1()
{
	return delta_roll;
}
uint16_t AP_Radio::chan2()
{
	return delta_pitch;
}
uint16_t AP_Radio::chan3()
{
	return delta_throttle;
}
uint16_t AP_Radio::chan4()
{
	return delta_yaw;
}
uint16_t AP_Radio::chan5()
{
	return delta_aux1;
}
uint16_t AP_Radio::chan6()
{
	return delta_aux2;
}
uint16_t AP_Radio::chan7()
{
	return delta_aux3;
}

// Since i have wired the Channel to A12 for now but normally this should return aux4 as wired to A15
uint16_t AP_Radio::chan8()
{
	return delta_aux1;
}

void AP_Radio::pin1func()
{
  if(digitalRead(MYPIN1)==HIGH)
	time_H = micros();
  else if(digitalRead(MYPIN1)==LOW)
	time_L = micros();
  
  // process roll
  if(time_L > time_H)
  {
	if(time_L-time_H < 2200)
	{
	  if(time_L-time_H > 750)
	  {
		delta_roll = time_L-time_H;
	  }
	}
  }
}


 void AP_Radio::pin2func()
{
  if(digitalRead(MYPIN2)==HIGH)
  pitch_time_H = micros();
  else if(digitalRead(MYPIN2)==LOW)
  pitch_time_L = micros();
  
  // process pitch
  if(pitch_time_L > pitch_time_H)
  {
	if(pitch_time_L-pitch_time_H < 2200)
	{
	  if(pitch_time_L-pitch_time_H > 750)
	  {
		delta_pitch = pitch_time_L-pitch_time_H;
	  }
	}
  }
}


 void AP_Radio::pin3func()
{
  if(digitalRead(MYPIN3)==HIGH)
  throttle_time_H = micros();
  else if(digitalRead(MYPIN3)==LOW)
  throttle_time_L = micros();
  
  // process throttle
  if(throttle_time_L > throttle_time_H)
  {
	if(throttle_time_L-throttle_time_H < 2400)
	{
	  if(throttle_time_L-throttle_time_H > 750)
	  {
		delta_throttle = throttle_time_L-throttle_time_H;
	  }
	}
  }
}


 void AP_Radio::pin4func()
{
  if(digitalRead(MYPIN4)==HIGH)
  yaw_time_H = micros();
  else if(digitalRead(MYPIN4)==LOW)
  yaw_time_L = micros();
  
  // process throttle
  if(yaw_time_L > yaw_time_H)
  {
	if(yaw_time_L-yaw_time_H < 2200)
	{
	  if(yaw_time_L-yaw_time_H > 750)
	  {
		delta_yaw = yaw_time_L-yaw_time_H;
	  }
	}
  }
}


 void AP_Radio::pin5func()
{
  if(digitalRead(MYPIN5)==HIGH)
  aux1_time_H = micros();
  else if(digitalRead(MYPIN5)==LOW)
  aux1_time_L = micros();
  
  // process throttle
  if(aux1_time_L > aux1_time_H)
  {
	if(aux1_time_L-aux1_time_H < 2200)
	{
	  if(aux1_time_L-aux1_time_H > 750)
	  {
		delta_aux1 = aux1_time_L-aux1_time_H;
	  }
	}
  }
}


 void AP_Radio::pin6func()
{
  if(digitalRead(MYPIN6)==HIGH)
  aux2_time_H = micros();
  else if(digitalRead(MYPIN6)==LOW)
  aux2_time_L = micros();
  
  // process aux2
  if(aux2_time_L > aux2_time_H)
  {
	if(aux2_time_L-aux2_time_H < 2200)
	{
	  if(aux2_time_L-aux2_time_H > 750)
	  {
		delta_aux2 = aux2_time_L-aux2_time_H;
	  }
	}
  }
}


 void AP_Radio::pin7func()
{
  if(digitalRead(MYPIN7)==HIGH)
  aux3_time_H = micros();
  else if(digitalRead(MYPIN7)==LOW)
  aux3_time_L = micros();
  
  // process aux3
  if(aux3_time_L > aux3_time_H)
  {
	if(aux3_time_L-aux3_time_H < 2200)
	{
	  if(aux3_time_L-aux3_time_H > 750)
	  {
		delta_aux3 = aux3_time_L-aux3_time_H;
	  }
	}
  }
}


 void AP_Radio::pin8func()
{
  if(digitalRead(MYPIN8)==HIGH)
  aux4_time_H = micros();
  else if(digitalRead(MYPIN8)==LOW)
  aux4_time_L = micros();
  
  // process aux4
  if(aux4_time_L > aux4_time_H)
  {
	if(aux4_time_L-aux4_time_H < 2200)
	{
	  if(aux4_time_L-aux4_time_H > 750)
	  {
		delta_aux4 = aux4_time_L-aux4_time_H;
	  }
	}
  }
}