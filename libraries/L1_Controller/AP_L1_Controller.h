#pragma once
#include "AP_AHRS.h"
#define PI_1	PI
class AP_L1
{
  public:
  AP_L1(AP_AHRS &_ahrs):
  ahrs(_ahrs)
  {
	cross_track 	= 0;
	bearing     	= 0;
	last_Nu         = 0;
	_target_bearing = 0;
  };
  
  float update_waypoint(Location prevWP, Location nextWP);
  float update_loiter(Location centerWP,  float radius);
  
  private:
  AP_AHRS &ahrs;
  
  float cross_track;
  float bearing;
  float last_Nu;
  float _target_bearing;
  void  sanity_check(float &Nu);
};
