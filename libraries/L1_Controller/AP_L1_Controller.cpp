#include "AP_L1_Controller.h"

void
AP_L1::sanity_check(float &Nu)
{
  float Nu_limit = 0.9f * 3.141962f;
  if( fabs(Nu) > Nu_limit
   && fabs(last_Nu) > Nu_limit
   && fabs(wrap_180(ToDeg(_target_bearing) - ToDeg(ahrs.yaw))) > 122
   && Nu * last_Nu < 0.0f)
   {
     Nu = last_Nu; // dont go this direction constrain Nu here
   }  
}

float AP_L1::update_waypoint(Location prevWP, Location nextWP)
{
  Location currentPos;  
  float Nu          = 0;
  float xtrackVel   = 0;
  float ltrackVel   = 0;
  float crossTrack  = 0;
  float nav_bearing = 0;
  float l1_damping  = AP_params.ParameterStorage.list.L1_damping;
  float l1_period   = AP_params.ParameterStorage.list.L1_period;
  float K_L1        = 4 * l1_damping * l1_damping;  //  Z = 1/sqrt(2) 
  
  // Update currentPosition and GroundSpeed Vectors
  ahrs.get_position(currentPos); 
  Vector2f groundSpeedVector = ahrs.groundspeed_vector();  
  _target_bearing = _NE_bearing(currentPos, nextWP);  
  float groundSpeed     = groundSpeedVector.length();
  
  // Check if groundspeed is too small and if so use yaw readings to define the vector
  if(groundSpeed < 1.0f)
  {
    groundSpeed = 1.0f;
    groundSpeedVector = Vector2f(cos(ahrs.yaw), sin(ahrs.yaw)) * groundSpeed;
  }
  
  // dynamically determine l1
  float l1_length = l1_period * groundSpeed * l1_damping * 1/PI_1;
   
  Vector2f AB     = _NE_distance(prevWP, nextWP);
  
  if(AB.length() < 1.0e-6f)
  {
    // if the distance between WPA and WPB is very small
    // dont do cross tracking, do direct tracking from
    // current position to WayPoint B.
    AB = _NE_distance(currentPos,nextWP);  
    
    // Now SInce We have Updated the Ab Matrix
    // agin check to see if AB length is Okay
    if(AB.length() < 1.0e-6f)
    {
        // Since we are directly ontop of WayPoint B as the distance is very small
        // Update AB Matrix into a New North-Easting Vector based on Yaw-Compass Readings
        AB = Vector2f(cos(ahrs.yaw), sin(ahrs.yaw));
    }      
  }
  
  // Normalise AB to obtain the unit vector of AB
  AB.normalise();

  // From A to Aircraft
  Vector2f A_Air = _NE_distance(prevWP, currentPos);
  
  //  This is Positive if Aircraft is to the left of AB
  crossTrack       = AB % A_Air;
  
  // along track distance
  float ltrackdist = A_Air * AB;
    
  /*@sanity check
   *@ if we satistfy these 2 conditions track to A instead of B else track to B (usual)
   *@ 1. if we are behind a +- 135 degree arc from AB direction
   *@ 2. if the distance to A is greater than the calculated l1 distance
   */
  float WP_A_dist = A_Air.length();
  if(WP_A_dist > l1_length && ltrackdist/fmax(WP_A_dist, 1.0f) < -0.7071)
  {
     Vector2f A_Air_unit  = (A_Air).normalized();  
     Vector2f Air_A_unit  = -A_Air_unit;
     xtrackVel   = groundSpeedVector % Air_A_unit;
     ltrackVel   = groundSpeedVector * Air_A_unit;
     Nu          = atan2(xtrackVel, ltrackVel);
     nav_bearing = atan2(Air_A_unit.y,Air_A_unit.x);
  }
  else
  {
     // calculate less constrained velocity capture angle 
     xtrackVel = groundSpeedVector % AB;
     ltrackVel = groundSpeedVector * AB;
     float Nu2 = atan2(xtrackVel, ltrackVel);    
     
     // calculate heavily constrained track capture angle to +- 45 degrees
     float xtrackErr = A_Air % AB;
     float sine_Nu1 = xtrackErr/ fmax(l1_length, 0.1f);
     sine_Nu1    = constrain_float(sine_Nu1, -0.7071f, 0.7071f);     
     float Nu1   = asin(sine_Nu1);     
     Nu          = Nu1 + Nu2;
     nav_bearing = atan2(AB.y, AB.x) + Nu; // bearing from Aircraft to L1 
  }
    
  /*@ Second sanity check
   *@ Prevent indecision when the following conditions are met
   *@ 1. Nu is greater than the limits  i.e. Nu > 0.9 * 3.141962 telling us we are moving away from target point but doesnt tell us if we are pointing away as this is a function of groundspeed vector and xtrackerr
   *@ 2. If we are pointing away given by the absolute difference between the _target_bearing and the current heading
   *@ 3. if we are oscillating given as the actual product between Nu and last Nu
   */
  float Nu_limit = 0.9f * PI_1;
  if( fabs(Nu) > Nu_limit && 
      fabs(last_Nu) > Nu_limit && 
      fabs(wrap_180(ToDeg(_target_bearing) - ToDeg(ahrs.yaw))) > 122 &&
      Nu * last_Nu < 0.0f)
   {
     Nu = last_Nu; // dont go this direction constrain Nu here
   }
   
   last_Nu = Nu;
   Nu = constrain_float(Nu, -PI_1/2, PI_1/2); // this means you can only deviate 90degrees from the desired path or perpendicular to the track
   
   float _latAccDem = K_L1 * groundSpeed * (groundSpeed / l1_length) * sin(Nu);
   return(_latAccDem);
}


float
AP_L1::update_loiter(Location centerWP, float radius)
{
  /*@ The logic for this part is simple
   *@ 1. If inside the circle radius:
   *@ : circle with centripetal acceleration
   *@ : go back to the track with PD acceleration from d and d_dot::
   *@ 2. If outside the circle radius:
   *@ : capture the desired track | loiter path with capture acceleration
   *@ : Note: Here: accel1 will be greater than 2 at this condition :-> always
   */
  float latAccDem;
  float wpRadius   = radius;
  float l1_damping = AP_params.ParameterStorage.list.L1_damping;
  float l1_period  = AP_params.ParameterStorage.list.L1_period;
  float K_L1       = 4 * l1_damping * l1_damping;
  
  Vector2f groundSpeedVector = ahrs.groundspeed_vector();
  float groundSpeed = fmax(groundSpeedVector.length(), 1.0f);
  Location currentLoc;
  ahrs.get_position(currentLoc); 
  
  float _target_bearing = _NE_bearing(currentLoc, centerWP);
  float l1_dist = l1_period * l1_damping * groundSpeed * 1/PI_1;
  
  // Vector from A to Aircraft
  Vector2f A_Air = _NE_distance(centerWP, currentLoc);
  Vector2f A_Air_unit;// =  (A_Air).normalized();
  if(A_Air.length() < 0.1f)
  {
    if(groundSpeedVector.length() < 0.1f)
    {
      A_Air_unit = Vector2f(cos(ahrs.yaw), sin(ahrs.yaw));
    }
    else
    {
      A_Air_unit = (groundSpeedVector).normalized();
    }
  }
  else
  {
    A_Air_unit = (A_Air).normalized();
  }
  
  // calculate capture angle::
  float xtrackVel = A_Air_unit % groundSpeedVector;
  float ltrackVel = -(groundSpeedVector * A_Air_unit);
  float Nu        = atan2(xtrackVel, ltrackVel);
  
  float Nu_limit = 0.9f * PI_1;
  if( fabs(Nu) > Nu_limit && 
      fabs(last_Nu) > Nu_limit && 
      fabs(wrap_180(ToDeg(_target_bearing) - ToDeg(ahrs.yaw))) > 122 &&
      Nu * last_Nu < 0.0f)
  {
     Nu = last_Nu; // dont go this direction constrain Nu here
  }
  last_Nu = Nu;
  
  // Limit Nu
  Nu = constrain_float(Nu, -PI_1/2, PI_1/2);
  
  // capture acceleration
  float latAccDemCap = K_L1 * groundSpeed * (groundSpeed/l1_dist) * sin(Nu);
  
  // PD Acceleration = d_dot_dot + 2*damp*wn*d_dot + wn^2 * d
  float XtrackVelCirc = -ltrackVel; // Radial outbound velocity
  float XtrackErrCirc = A_Air.length() - wpRadius;
  float omega = (PI_1 * 2) / l1_period;    // natural frequency of the controller::
  float kv = 2 * l1_damping * omega; // crosstrack_dot coefficient
  float kx = omega * omega;          // crosstrack coefficient
  float latAccDemCircPD = kv*XtrackVelCirc + kx*XtrackErrCirc;
  float VelTangent = xtrackVel;
  
  // if moving away from center limit PD to positive values only
  if(ltrackVel < 0 && VelTangent < 0)
  {
    latAccDemCircPD = fmax(latAccDemCircPD, 0);
  }
  
  // 
  float radius_curr       =  wpRadius + XtrackErrCirc;
  float radius_lim        = 0.5f*wpRadius;
  float latAccDemCircCntr = VelTangent * VelTangent / fmax(radius_lim,radius_curr);
  float latAccDemCirc = latAccDemCircPD + latAccDemCircCntr;
  
  if(XtrackErrCirc > 0 && latAccDemCap < latAccDemCirc)
  {    
    latAccDem = latAccDemCap; // needs to be constrained to the direction of loiter wanted...
    float nav_bearing = atan2(-A_Air_unit.y, -A_Air_unit.x);
  }
  else
  {
    latAccDem = latAccDemCirc;
    float nav_bearing = atan2(-A_Air_unit.y, -A_Air_unit.x);    
  }
  
  return latAccDem;
}
