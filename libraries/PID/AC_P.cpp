#include "AC_P.h"
float AC_P::get_p(float input){
	
	float kp = *(gains.tau);	
	kp 		 = 1/kp;
	return kp*input;
}