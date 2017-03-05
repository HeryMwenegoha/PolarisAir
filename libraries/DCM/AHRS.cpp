#include "AHRS.h" 
void 
AP_AHRS_BASE:: update_degress()
{
	rollDeg  = ToDeg(roll);
	pitchDeg = ToDeg(pitch);
	yawDeg   = ToDeg(yaw);
	
	if(yawDeg < 0)
		yawDeg += 360;
}




