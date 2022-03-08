#ifndef Common_h
#define Common_h
#include "Arduino.h"
#include "Vectors.h"
#include "mavlink.h"


// mavlink_message_t msg;
// mavlink_status_t  status;
// uint8_t 		  buf[128];  
  

/* In the Update Function of Each Sensor, Use the Instance in the Enum Below */
/* Changing the order of the enum changes */
#define I2CSPEED 100000

enum Sens
{
	L3Gd20 = 0,
	MPU6000,
	L3Gd20H,
	FXAS21002
};

enum Accel
{
	LSM303A = 0,
	MPU6000A,
	L3Gd20HA,
	FXOS8700A
};

enum Compass{
	LSM303D = 0,
	HMC5883,
	AK8963,
	FXOS8700
};

/* Follows the mavlink MAV_CMD enum and therefore command can be used directly
 */
enum Flight_Stage{	
	FLIGHT_WAYPOINT			=16, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */
	FLIGHT_LOITER_UNLIM 	=17, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	FLIGHT_LOITER_TURNS 	=18, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	FLIGHT_LOITER_TIME  	=19, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	FLIGHT_RETURN_TO_LAUNCH =20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	FLIGHT_LAND				=21, /* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	FLIGHT_TAKEOFF			=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
	FLIGHT_ROI				=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
	FLIGHT_PATHPLANNING		=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
	FLIGHT_LAST				=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	FLIGHT_DELAY			=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */	
	FLIGHT_LAND_APPROACH    =23   // Not in Mavlink Definition
};

inline float constrain_float(float val, float min, float max)
{
	if(val > max)
	val = max;
	else if(val < min)
	val = min;
	
	return val;
}

inline float map_float(float val, float valmin, float valmax, float outmin, float outmax)
{
	if(val > valmax){
		val = valmax;
	}else if(val < valmin){
		val = valmin;
	}
	
	float dex = ((val - valmin)/(valmax - valmin)) * (outmax - outmin) + outmin;
	
	return dex;
}



inline float ToRad(float x)
{
	return x*0.01745329;
}

inline float radiansf(float x)
{
	return x*0.01745329;
}

inline float radiansf()
{
	return 0.01745329;
}

inline float ToDeg(float x)
{
	return x*57.2957795;
}

inline float degreesf(float x)
{
	return x*57.2957795;
} 

inline float degreesf()
{
	return 57.2957795;
} 

inline float wrap_360(const float deg)
{
	float res = fmod(deg, 360.0f);
	if(res < 0){
		res += 360.0f;
	}		
	return res;
}

inline float wrap_180(const float deg)
{
	float res = wrap_360(deg);
	if(res > 180.0f){
		res -= 360.0f;
	}
	return res;
}

typedef uint8_t    AP_u8;
typedef int8_t     AP_i8;
typedef uint16_t   AP_u16;
typedef float      AP_float;
typedef double     AP_double;

struct Location{
	public:
	/*
	Location()
	{
	  lat = 0;
	  lon = 0;
	  alt = 100;
	  rad = 20;
	  seq = 1;   // offset home
	  cmd = 16;  // waypoint command
	}
	*/
	
	float    lat;
	float    lon;
	uint16_t alt;
	uint8_t  rad;
	uint8_t  seq;
	uint8_t  cmd;
	
	/*
	Location(float lati, float longi)
	{
		this->lat = lati;
		this->lon = longi;
		this->alt = 100;
		this->rad = 20;
		this->seq = 0;
		this->cmd = 0;
	}
    
	Location()
	{
	  lat = 0;
	  lon = 0;
	  alt = 100;
	  rad = 20;
	  seq = 1;   // offset home
	  cmd = 16;  // waypoint command
	}
	*/

	bool isvalid(){
	  if(isnan(lat) || isnan(lon))
		return false;
	  else if(isinf(lat) || isinf(lon))
		return false;
	  else
		return true;
	}

	void zero(){
	  this->lat = 0;
	  this->lon = 0;
	  this->alt = 100;
	  this->rad = 20;
	  this->seq = 1;
	  this->cmd = 16;
	}

	void print(){
	  Serial.print(this->lat,7);
	  Serial.print(F("\t"));
	  Serial.print(this->lon,7);
	  Serial.print(F("\t"));
	  Serial.print(this->alt);
	  Serial.print(F("\t"));
	  Serial.print(this->rad);
	  Serial.print(F("\t"));
	  Serial.print(this->seq);
	  Serial.print(F("\t"));
	  Serial.println(this->cmd);
	}

};


/*
struct Location
{
  public:
  float lat;
  float lon;
  
  Location(float lati, float longi)
  {
    this->lat = lati;
    this->lon = longi;
  }
  
  Location()
  {
    this->lat = 0.0f;
    this->lon = 0.0f;
  }
};
*/

struct Position3f 
{
  float lat;
  float lon;
  float alt;
  uint16_t cmd;
  
  Position3f(float lati, float longi, float alti, uint16_t _cmd)
  {
    lat = lati;
    lon = longi;
    alt = alti;
    cmd = _cmd;
  };
};


inline Vector2f _NE_distance(Location prevWP, Location nextWP)
{
   float R = 6371000; //  earths radius in metres
   return(Vector2f((nextWP.lat - prevWP.lat)*(PI/180) * R, ( nextWP.lon - prevWP.lon)*(PI/180) * R * cos(ToRad(prevWP.lat))));
}


inline float _NE_bearing(Location prevWP, Location nextWP)
{
   Vector2f _N_E = _NE_distance(prevWP, nextWP);
   float theta   = atan2(_N_E.y, _N_E.x);
   return theta;
}

inline Vector2f _NE_distance_offset(Location prevWP, Location nextWP, float offset)
{
   float R = 6371000;
   Vector2f _N_E = _NE_distance(prevWP, nextWP);
   float theta   = atan2(_N_E.y, _N_E.x);   
   return(Vector2f((nextWP.lat - prevWP.lat)*(PI/180) * R + offset * cos(theta), ( nextWP.lon - prevWP.lon)*(PI/180) * R * cos(ToRad(prevWP.lat)) + offset * sin(theta)));
}

/*
inline Location _NE_bearing(Location prevWP, float bearing_offset)
{
   float R = 6371000;
   Vector2f _N_E = _NE_distance(prevWP, nextWP);
   float theta   = atan2(_N_E.y, _N_E.x);  
   
   float distance = offset;
   float theta    = bearing;
   
   float x = offset * cos(theta);
   float y = offset * sin(theta);
   
  
   return(Vector2f((nextWP.lat - prevWP.lat)*(PI/180) * R + offset * cos(theta), ( nextWP.lon - prevWP.lon)*(PI/180) * R * cos(ToRad(prevWP.lat)) + offset * sin(theta)));
}
*/

inline void location_offset(Location &WP, float offset_north, float offset_east)
{
   float R 				   = 6371000;
   float R_lon             = R * cos(ToRad(WP.lat));
   float north_deg_offset  = (180/PI) * (offset_north/R);			 // degrees offset in north..
   float east_deg_offset   = (180/PI) * (offset_east/R_lon);   
   WP.lat 				  += north_deg_offset;
   WP.lon 				  += east_deg_offset;
}

inline float safe_sqrt(float x){
	if(x < 0)
		return 0;
	
	else 
		return sqrt(x);
}

#endif