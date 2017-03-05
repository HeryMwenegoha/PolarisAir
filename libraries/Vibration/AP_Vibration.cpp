#include "AP_Vibration.h"
#include "Common.h"

vector3f AP_Vibration::update(vector3f _accel_in){
	uint64_t now = micros();
	float  dt    = (now - _last_usec) * 1e-6f;
	_last_usec   = now;
	
	vector3f  accel = _accel_in;
	
	if(dt > 1.0f){
		// reset filter
		accel_last  = accel;
		dt          = 0.02; // smaller time constant
		accel_floor = accel;
		accel_diff_last_sq.zero();
		vibe_sq.zero();
	}
	
	if(VIBE_FLOOR_HPF_HZ  < 1.0f)
		VIBE_FLOOR_HPF_HZ = 1.0f;
	
	// Filter Floor at 5Hz
	float RC       = 1/(2 * PI * VIBE_FLOOR_HPF_HZ);
	float beta     = RC/(RC + dt);		
	vector3f temp  = accel_floor          * beta;
	vector3f _temp = (accel - accel_last) * beta;
    accel_floor    = temp + _temp;
	accel_last     = accel;
	
	vector3f accel_diff_sq = accel - accel_floor;
	accel_diff_sq.x       *= accel_diff_sq.x;
	accel_diff_sq.y       *= accel_diff_sq.y;
	accel_diff_sq.z       *= accel_diff_sq.z;
	
	// Filter Squares at 2Hz
	if(VIBE_DIFF_HPF_HZ < 1.0f)
		VIBE_DIFF_HPF_HZ = 1.0f;
	RC      = 1/(2 * PI * VIBE_DIFF_HPF_HZ);
	beta    = RC/(RC + dt);		
    temp    = vibe_sq          * beta;
	_temp   = (accel_diff_sq - accel_diff_last_sq) * beta;
    vibe_sq = temp + _temp;
	accel_diff_last_sq  = accel_diff_sq;
	
	// Calculate the squareroot:
	vector3f vibe = vibe_sq;
	vibe.x = safe_sqrt(vibe.x);
	vibe.y = safe_sqrt(vibe.y);
	vibe.z = safe_sqrt(vibe.z);	

	return vibe;
}