#include "AP_BatteryVolts.h"

void AP_BatteryVolts::initialise()
{
	analogRead(A0); // throw away
	delay(3);		// stability
	float adc = 0;
	for(int i = 0; i < 5; i++){
		adc += analogRead(A0);
		delay(3);
	}
	
	volts = adc/5;
}


// 10Hz read 1Hz update	
void AP_BatteryVolts::read(){
	uint16_t val = analogRead(A0);
	sum_adc  += val;
	count++;
	
	if(count == 10){
		float raw_volts = (float)sum_adc/count;
		sum_adc   = 0;
		count     = 0;
		
		volts     = 0.7 * volts + 0.3 * raw_volts;
	}
}

float AP_BatteryVolts::get_adc(){
	return volts;
}