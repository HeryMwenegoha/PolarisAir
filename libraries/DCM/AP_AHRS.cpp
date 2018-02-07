#include "AP_AHRS.h"
void  
AP_AHRS::update()
{
	if(last_startup_Msec == 0)
		last_startup_Msec = millis();
		
	uint32_t t_now = millis();
    uint32_t dt	   = t_now - update_Msec;
	G_dt 		   = (float)dt * 0.001f; 
	update_Msec    = t_now;
	
	if(G_dt > 0.2f)
	{
		_ra_sum.zero();
		_ra_deltat = 0;
		return;
	}
		
	update_matrix();
	normalise();
	drift_correction();
	check_matrix();
	euler_angles();	
	
	// calculate vibrations
	vibe = AP_vibration.update(acc);
	
	// if in hill mode and no new update for the past 5 seconds
	// reset everything, that means fcs is on and in hil mode nut no data from xplane
	if(_hil_mode)
	{
		if(millis() - _last_hil_millis >= 5000)
		{
			//Serial.println("Reset");
			AP_AHRS::reset_dcm(false); // dont recover
		}
		return;
	}
}



void 
AP_AHRS::update_matrix()
{ 

  gyro_vector.zero();
  accel_vector.zero();
  byte _gyro_reg  = 0;
  byte _accel_reg = 0;
  for(int i = 0; i <3; i++)
  {
	if(_ins->_gyro_health_count[i]){
		_gyro_reg++;
		gyro_vector = gyro_vector + _ins->gyro()[i];   	// rad/s
	}
	
	if(_ins->_accel_health_count[i]){
		_accel_reg++;
		accel_vector = accel_vector + _ins->accel()[i];  // m/s2
	}
  }
  
  if(_gyro_reg< 1)
	_gyro_reg = 1;

  if(_accel_reg < 1)
	_accel_reg = 1;

  gyro_vector  = gyro_vector/_gyro_reg;
  accel_vector = accel_vector/_accel_reg;
  acc = accel_vector;
  
  //acc.print();
  // accel_vector.printV();
  // (gyro_vector * degreesf()).printV();
  
  
  /*
  Serial.print(_ins->raw_accel().x);
  Serial.print("    ");
  Serial.print(_ins->raw_accel().y);
  Serial.print("    ");
  Serial.println(_ins->raw_accel().z);
  */
  
  omega.zero();
  omega 	 = gyro_vector  + omega_I;  
  dcmMatrix.rotate((omega + omega_P + omega_yaw_P) * G_dt);
}

bool 
AP_AHRS::renormalise(vector3f const &a, vector3f &result)
{
  float renorm_val = 0;
   renorm_val  = 1.0f/a.magnitude();  
   if(!(renorm_val < 2.0f && renorm_val > 0.5f))
   {
      // outside of range values
      if(!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f))
      {
        return false;
      }
   }   
   result = a * renorm_val;
   return true;
}

void 
AP_AHRS::normalise()
{
  // Correct for othorgonality conditions
  vector3f x_orth, y_orth, z_orth;
  float error = 0;
 
  error  = dcmMatrix.a * dcmMatrix.b;
  x_orth = dcmMatrix.a - (dcmMatrix.b * (0.5f * error));
  y_orth = dcmMatrix.b - (dcmMatrix.a * (0.5f * error));
  z_orth = x_orth % y_orth;
  
  if(!renormalise(x_orth, dcmMatrix.a) ||
     !renormalise(y_orth, dcmMatrix.b) ||
     !renormalise(z_orth, dcmMatrix.c)){
           AP_AHRS::reset_dcm(true);
           last_failure_Msec = millis();
     }
}


float 
AP_AHRS::yaw_error_compass()
{
}

float 
AP_AHRS::_P_gain(float _spin_rate) // spinnrate in radians/sec as omega.length
{
	if(_spin_rate < ToRad(50))
		return 1.0f;
	if(_spin_rate > ToRad(500))
		return 10.0f;
	return _spin_rate/ToRad(50);
}

float 
AP_AHRS::_yaw_gain(void) const
{
	/*
	float vdot_ef = sqrt(_accel_ef.x * _accel_ef.x + _accel_ef.y * _accel_ef.y);	
	if(vdot_ef <= 4.0f)
		return 0.2f*(4.5f - vdot_ef);
	return 0.1f;
	*/
}

bool  
AP_AHRS::use_fast_gains(void)const
{
	return (millis() - last_startup_Msec) < 15000U;
}

bool  
AP_AHRS::use_compass()
{
	// I dont have a compass object or compass has not been detected
	if(!_compass || !_compass->have_compass())
		return false;
		
	// We dont have GPS and we are not flying forward, no alternative here use compass
	if(!_gps.have_gps() || !_flags.fly_forward)
		return true;
		
	// Too slow to rely on GPS for compass correction, no choice but to use the compass
	if(_gps.groundspeed() < GPS_SPEED_MIN)
		return true;
		
	// if yaw differs from ground course by more than 45 degress
	// and estimated wind is less than 80% of ground speed switch to GPS
	// very bad compass offsets
	// Otherwise report consistent heading
	float error = fabs(wrap_180(_gps.heading() - yawDeg));
	if(error > 45.0f && wind.length() < _gps.groundspeed() * 0.8f)
	{
		// 2 seconds since last report of consistent heading then declare that compass is not to be used
		// We can therefore declare the compass is not to be trusted
		if(millis() - _last_consistent_heading > 2000)
		{
			return false;
		}
	}
	else
	{
		_last_consistent_heading = millis();
	}
	
	// if all the above didnt catch then just use heading
	return true;
}

vector3f AP_AHRS::_ra_delayed(vector3f new_ra)
{
	// delays the acceleration by 200 ms to match the UBX delay.
	vector3f delayed_vector = _ra_delayed_buffer;
	_ra_delayed_buffer  = new_ra;
	if(delayed_vector.is_zero())// initial start.
		return new_ra;	
	return delayed_vector;
}


// drift correction using gps or compass
// omega_yaw_p vector is produced here
void 
AP_AHRS::drift_correction_yaw(void)
{
	// Compass Correction
	bool new_value = false;
	float yaw_dt = 0; 
	float _error_course = 0;
	if(_compass && use_compass())
	{
		if(_compass->last_update_msec() != _compass_last_update_msec)
		{
			yaw_dt 	= static_cast<float>(_compass->last_update_msec() - _compass_last_update_msec) * 1.0e-3f;
			_compass_last_update_msec   = _compass->last_update_msec();
			float heading  = _compass->calculate_heading(roll, pitch);
			if(!_flags.have_initial_yaw){
				_flags.have_initial_yaw = true;
				omega_yaw_P.zero();
			}
			// yaw error by compass
			Vector2f _b_field     = Vector2f(cos(heading), sin(heading));
			Vector2f _dcm_b_field = Vector2f(dcmMatrix.a.x, dcmMatrix.b.x);
			
			_error_course   	  = _dcm_b_field % _b_field; // sin of angle from dcm_b_field to heading field
			
			new_value = true;
		}
	}else if(_flags.fly_forward && _gps.have_gps()){
			
			if(_gps.last_time_fix_ms() != _gps_last_update_msec &&
			   _gps.groundspeed()      >= GPS_SPEED_MIN){
				yaw_dt 	= static_cast<float>(_gps.last_time_fix_ms() - _gps_last_update_msec) * 1.0e-3f;
				_gps_last_update_msec = _gps.last_time_fix_ms();
				if(!_flags.have_initial_yaw){
					_flags.have_initial_yaw = true;
					omega_yaw_P.zero();
				}	
				// yaw error by GPS
				_error_course = sin(ToRad(_gps.heading()) - yaw); // FRom yaw to gps heading
				
				new_value = true;
			}
	}

	if(!new_value)
	{
		omega_yaw_P = omega_yaw_P * 0.97; // decay the yaw term
		return;
	}
	
	vector3f _error_yaw_ef = vector3f(0,0,_error_course);
	vector3f _error_yaw    = dcmMatrix.mul_transpose(_error_yaw_ef);		// Transform into rotations in the body frame
	_error_yaw.x = _error_yaw.y = 0; 									// only rotate around z-axis
	
	if(KP_YAW < KP_YAW_MIN){
		KP_YAW  = KP_YAW_MIN;
	}
	
	float _spin_rate = omega.length();
	
	omega_yaw_P = _error_yaw * KP_YAW * _P_gain(_spin_rate); // update the yaw term
	
	if(use_fast_gains()){
		omega_yaw_P.z *= 8; 
	}
	
	if(yaw_dt < 2.0 && _spin_rate < ToRad(SPIN_RATE_LIMIT)){
		omega_I_sum = omega_I_sum + _error_yaw * KI_YAW * yaw_dt;
	 }		 	
}

void AP_AHRS::drift_correction()
{	
	uint32_t last_correction_time;
	vector3f velocity;
	vector3f _accel_ef = dcmMatrix * accel_vector; 		 // m/s2 => gravity in body frame	
	_ra_sum     = _ra_sum + _accel_ef  * G_dt;			     // RAdt	
	_ra_deltat += G_dt;
	
	// internally corrects for heading using the Isum term and omega_yaw_P terms
	drift_correction_yaw();
	
	// if we dont have GPS then we switch to accelerometer correction
	// conventional correction method used in DCM fixed wing
	if(!_gps.have_gps() 					|| 
	_gps.status() < AP_GPS::GPS_FIX_TYPE_3D || 
	_gps.num_sats() < AP_GPS::GPS_MIN_SATS)
	{
		if(_ra_deltat < 0.2f)
		{
			return;									 // make sure we perform corrections at 5Hz
		}
		float airspeed = 3;							     // Here we assume airspeed is more valid and we will use this to estimate groundspeed|velocity for dead reckoning navigation
		if(_airspeed->enabled()){
			airspeed = _airspeed->get_airspeed();    // we have a valid airspeed reading from a sensor then we use this airspeed value
		}
		else{
			airspeed = _last_airspeed;			     // if we dont have a valid airspeed sensor reading we will use the value of _last_airspeed which is updated once we have a valid GPS reception as our airspeed
		}
		
		velocity     = dcmMatrix.colx() * airspeed;  // airspeed in earth frame
	
		velocity     = velocity + wind;			     // estimated ground velocity
		
		_have_gps_lock = false;						 // we remember this so that when we do get GPS reception we first update our last velocity before we proceed if not we end up using an uncorrected velocity with a proper velocity
		last_correction_time = millis(); // difference of 0.2 seconds
	}
	else
	{
		if(_gps.last_time_fix_ms() == _ra_sum_start)
		{
			return;								// Make sure we perform corrections at a minimum of 5Hz -> can sometimes even be 1Hz -> during this time _ra_deltat will be accumulating for the integral
		}
		velocity = _gps.velocity();				// Here we assume the velocity is far more accurate than airspeed and we use the velocity to update last airspeed for later on when we dont have GPS reception
		
		if(_have_gps_lock == false)
		{
			_last_velocity = velocity; 			// if we didnt have gps in last drift correction then set velocity equal to last velocity
		}
		vector3f airspeed  = velocity - wind;
		airspeed.z 		   = 0.0f;				// dont need z component of airpseed
		_last_airspeed     = airspeed.length();	// if we dont have an airspeed indicator enabled then we can use this value of airspeed after we loose gps reception
		
		_have_gps_lock = true;		
		last_correction_time = _gps.last_time_fix_ms(); // last correction time will be used to update _ra_sumTime-> set to gps fix time in millis of fix.
	}
	
	// If we have initialised GPS sensor and our fix status is atleast dead-reckoning
	// Then we can safely update our last latitude and longitude and set offsets to 0
	// If not then we will use our estimated velocity to workout the position offsets from last latitude and longitude
	// In our L1 controller we will check if we have GPS and if we dont then we will switch to DR
	// in DR:: we need to return our estimated location by having a function that adds the offset converted into latitude and longitude degrees to last latitude and longitude
	if(_gps.have_gps())
	{
		_last_lat = _gps.location().lat;
		_last_lon = _gps.location().lon;
		_position_offset_north = 0.0f;
		_position_offset_east  = 0.0f;
		_have_position = true;        					   // we know exaclty the position so obviously yes
	}
	else
	{
		_position_offset_north += velocity.x * _ra_deltat; // use estimated velocity from airspeed (better if we have an airspeed sensor while we have fixed our wind estimate) to workout offsets in north
		_position_offset_east  += velocity.y * _ra_deltat; // use estimated velocity to workout offsets in east
		_have_position 		    = false; 				   // we know exaclty the position so obviously yes
	}
	
	if(_ra_sum_start == 0)
	{
		_ra_sum_start = last_correction_time;
		_last_velocity = velocity;
		return;
	}
	
	if(_ra_deltat <= 0.0f)
	{
		return; // just remove this, doesnt make any sense here
	}
	
	vector3f GA_b(0.0f, 0.0f, 0.0f); 
	vector3f GA_e(0.0f, 0.0f,-1.0f); 				// NED
	float _ra_scale = 1.0f/(9.81f * _ra_deltat);
	float gps_gain  = 1.0f; 						// How much to use GPS to correct for attitude
	float _v_scale  = gps_gain * _ra_scale;
	bool  using_gps_correction = false;
	
	// If we want to correct for centrifugal forces -> we actually do **DCM basis equation 19 i think**
	// and we either have a GPS lock (sure of our velocities) or just simply we know that we are flyig forward
	// Flying foward should be set to false in a quadplane in VTOL mode
	// If no GPS lock and no airspeed then recall that last_airspeed will be used to estimate ground velocity which will have a constant magnitude but variable direction i.e. dcmMatrix.colx() * _last_airspeed and therefore centrifugal corrections will roughly be okay
	// Will also happen is classical fixed wing accelerometer correction technique
	if(_flags.correct_centrifugal && (_have_gps_lock || _flags.fly_forward))
	{
		vector3f vdelta = (velocity - _last_velocity) * _v_scale;
		vdelta.z = constrain_float(vdelta.z,-0.5, 0.5);
		GA_e = GA_e + vdelta;
		GA_e.normalise();
		if(GA_e.is_inf())
		{
			last_failure_Msec = millis();
			return;
		}
		using_gps_correction = true;		// Whether we have GPS or not this will be set to true so long as we want to correct centrifugal forces as wel as we are flying forward.
	}
	
	
	// we can now calculate the error in the earth frame
	// error is calculate as the cross product of the normalised integrated Acceleromter projected into earth frame and reference earth gravity vector corrected for forward acceleration and centrifugal acceleration
	// since the rotational error equivalent to sin(theta) is in the earth frame, it needs to be transformed into the body frame to apply the rotation on the gyro vectors
	_ra_sum = _ra_sum * _ra_scale;
	
	if(using_gps_correction == true){
		GA_b = _ra_delayed(_ra_sum); 	  // use previous delayed ra to match incoming gps lag
	}else{
		GA_b = _ra_sum;
	}
	
	float length = GA_b.length();
	if(length > 1.0){
		GA_b.normalise();		
		if(GA_b.is_inf()){
			return;
		}
	}
	
	vector3f  _error_vector = GA_b % GA_e; 				
	
	// reduce the effect of 2 competing correction controllers on yaw
	if(_gps.have_gps() && is_equal(gps_gain, 1.0f)){
		_error_vector.z *= sin(fabs(roll));
	}else{
		_error_vector.z = 0.0f;
	}
	
	// Transpose back the error from the earth frame to the body frame
	_error_vector  = dcmMatrix.mul_transpose(_error_vector); 	
	
	
	// sanity check to make sure that error vector is pure
	// no infinities and nans
	if(_error_vector.is_nan() || _error_vector.is_inf()){
		check_matrix();
		last_failure_Msec = millis();
		return;
	}
	
	// Scale the Proportional constant with a spin rate capped at 50Deg/Sec
	// The value of spin rate should be in radianspersec
	float spin_rate = omega.length(); // rad/ses
	if(KP_ROLLPITCH < KP_ROLLPITCH_MIN){
		KP_ROLLPITCH = KP_ROLLPITCH_MIN;
	}
	omega_P  = _error_vector * KP_ROLLPITCH * _P_gain(spin_rate);	// correction to add to dcm to take it to accel.
	
	// We just powered our flight control system or
	// We just had a dcm reset following a failed check.
	// This speeds up the process of getting towards the desired angle
	if(use_fast_gains()){
		omega_P  = omega_P * 7.5;
	}
	
	// Assume that we are in take-off mode presumbaly LAUNCH ACCELERATION
	// We have a GPS fix -> atleast 2D but our ground speed is less than 3m/s
	// Our axial acceleration is greater than 6m/s2
	// Our pitch attitude is within +/- 3 degrees
	// The effect of GPS lag is reduced on correction term
	if(_flags.fly_forward	    				  && 
	_gps.status()      >= AP_GPS::GPS_FIX_TYPE_2D &&
	_gps.groundspeed() <= GPS_SPEED_MIN		      &&
	accel_vector.x     >= 7.0f					  &&
	pitchDeg > -3.0f  && pitchDeg < 3.0f){
		omega_P   = omega_P * 0.5f;
	}
		
	if(spin_rate < ToRad(SPIN_RATE_LIMIT)){
		omega_I_sum = omega_I_sum + _error_vector * KI_ROLLPITCH * _ra_deltat;
		omega_I_sum_time += _ra_deltat; 
	}
	
	if(omega_I_sum_time >= 5.0f){
		float change_limit  = 0.008722 * omega_I_sum_time;
		omega_I_sum.x = constrain_float(omega_I_sum.x, -change_limit, change_limit);
		omega_I_sum.y = constrain_float(omega_I_sum.y, -change_limit, change_limit);
		omega_I_sum.z = constrain_float(omega_I_sum.z, -change_limit, change_limit);
		omega_I 	  = omega_I + omega_I_sum;
		omega_I_sum_time    = 0;
		omega_I_sum.zero();
	}
	
	_ra_sum.zero();
	_ra_deltat     = 0;
	_ra_sum_start  = last_correction_time;
	_last_velocity = velocity;
	
	if(_have_gps_lock && _flags.fly_forward)
	{
		wind_estimate();  // with GPS will happen every 0.2 seconds
	}
}

void AP_AHRS::reset_dcm(const boolean recover_eulers)
{
   omega.zero();
   omega_P.zero();
   omega_I.zero();
   omega_yaw_P.zero();
   
   if(recover_eulers && 
      !isnan(roll) &&
      !isnan(pitch) && 
      !isnan(yaw)){
      dcmMatrix.from_euler(roll, pitch, yaw);
   }
   else
   {
     dcmMatrix.identity();
   }
   last_startup_Msec = millis();
}

void AP_AHRS::check_matrix(void)
{
  if(dcmMatrix.is_nan())
  {
    AP_AHRS::reset_dcm(true);
  } 
  if(!(dcmMatrix.c.x < 1.0f && dcmMatrix.c.x > -1.0f))
  {
    normalise();
    
    if(dcmMatrix.is_nan() || fabs(dcmMatrix.c.x) > 10)
    {
      // normalisation didnt fix the error
      AP_AHRS::reset_dcm(true);
    }
  }
}

void AP_AHRS::euler_angles()
{
    pitch 	  = -asin(dcmMatrix.c.x);                  // rad  -pi/2 | pi/2
    roll      =  atan2(dcmMatrix.c.y, dcmMatrix.c.z);  // rad  -pi 	| pi
    yaw    	  =  atan2(dcmMatrix.b.x, dcmMatrix.a.x);  // rad  -pi 	| pi
    
	update_degress();
	
    rollrate  = omega.x;  // rad/sec
    pitchrate = omega.y;  // rad/sec
    yawrate   = omega.z;  // rad/sec
}



void AP_AHRS::wind_estimate(void)
{
	vector3f gSpeed   = _last_velocity;
	vector3f fusDir   = dcmMatrix.colx();
	vector3f delta_fusDir = fusDir - fusDir_old;

	uint32_t now    = millis();
	if(now - _last_wind_time > 10000)
	{
		_last_wind_time = now;
		fusDir_old	    = fusDir;
		gSpeed_old 	    = gSpeed;
		return;
	}

	float fusMag    = delta_fusDir.magnitude();
	// Significant change
	if(fusMag > 0.2f)
	{
		vector3f delta_gSpeed = gSpeed - gSpeed_old;
		float gSpdMag 		  = delta_gSpeed.magnitude();

		float est_speed  	  = gSpdMag/fusMag;
			
		float theta      = atan2(delta_gSpeed.y, delta_gSpeed.x) - atan2(delta_fusDir.y, delta_fusDir.x); 	
		float sintheta   = sin(theta);
		float costheta   = cos(theta);

		vector3f _wind = vector3f();
		_wind.x = ((gSpeed.x + gSpeed_old.x) - est_speed *((cos(theta)*(fusDir.x + fusDir_old.x)) - sin(theta) * (fusDir.y + fusDir_old.y))) * 0.5;
		_wind.y = ((gSpeed.y + gSpeed_old.y) - est_speed *((sin(theta)*(fusDir.x + fusDir_old.x)) + cos(theta) * (fusDir.y + fusDir_old.y))) * 0.5;
		_wind.z = ((gSpeed.z + gSpeed_old.z) - est_speed *(fusDir.z + fusDir_old.z)) * 0.5;
		_wind   = _wind * WIND_ESTIMATE_TRUST;
		
		// Only update wind if the change that has occurred is less 20m/s of the previous wind
		// wind - previous wind 
		//_wind - current wind
		if(_wind.magnitude() < wind.magnitude() + 20)
		{
		  wind = wind * 0.95f + _wind * 0.05f;
		}
		gSpeed_old      = gSpeed;
		fusDir_old      = fusDir;
		_last_wind_time = now;
	}
	else if(now - _last_wind_time > 2000 &&	_airspeed->enabled())
	{
		vector3f _airspeed_ef = dcmMatrix.colx() * _airspeed->get_airspeed();
		vector3f _wind	= gSpeed - _airspeed_ef;
		wind = wind * 0.92f + _wind * 0.08f;
	}
}
/* @ Alternative method to calculate theta	
delta_fusDir.z = 0.0f;
delta_gSpeed.z = 0.0f;	
delta_fusDir.normalise();
delta_gSpeed.normalise();	
Vector2f _fx = Vector2f(delta_fusDir.x, delta_fusDir.y);
Vector2f _gx = Vector2f(delta_gSpeed.x, delta_gSpeed.y);
float _theta = asin(_fx % _gx); // from fx to gx
Serial.print("Theta:  "); Serial.print(theta); Serial.print(" ** ");Serial.println(_theta);
*/

float  AP_AHRS::altitude_estimate(){
	
	float altitude = 0;
	
	if(_baro.have_sens() && _baro.healthy()){
		// Barometer Height
		altitude =  _baro.get_altitude();
	}
	else if(_have_gps_lock && _flags.fly_forward){
		// GPS altitude
		altitude =  _gps.altitude();
	}
	
	return altitude;
}


float AP_AHRS::airspeed_estimate()
{	
	bool ret    = false;
	float speed = 0;
	
	// We have enabled Airspeed Sensor 
	// No need to go further than this
	if(_airspeed->enabled()){
		speed = _airspeed->get_airspeed();
		speed = constrain_float(speed, 2.0f, 40.0f);
		return speed;
	}
	
	// Estimate using GPS and wind -> compliments the airspeed sensor
	// Using wind estimate and gps to estimate airspeed
	if(_gps.have_gps()){
		speed = _last_airspeed;
		ret = true;
	}
	
	// If windmax is zero then airspeed estimate will be used as is::
	if(ret && WIND_MAX > 0 && _gps.state() == AP_GPS::GPS_FIX_OK)
	{
		float gnd_speed = _gps.groundspeed();
		speed 			= constrain_float(speed, 
								gnd_speed - WIND_MAX, 
								gnd_speed + WIND_MAX);
	}
	
	// convenience clipping of airspeed value
	speed = constrain_float(speed, 3.0f, 40);

	return speed;		
}

bool AP_AHRS::get_position(Location &loc)const
{
	loc.lat = _last_lat;
    loc.lon = _last_lon;
    //loc.alt = _baro.get_altitude() * 100 + _home.alt;
    //loc.flags.relative_alt = 0;
    //loc.flags.terrain_alt = 0;
    location_offset(loc, _position_offset_north, _position_offset_east);
	
	/* To be implemented in the future
    if (_flags.fly_forward && _have_position) {
        location_update(loc, _gps.ground_course_cd() * 0.01f, _gps.ground_speed() * _gps.get_lag());
    }
	*/
	
    return _have_position;
}

Vector2f AP_AHRS::groundspeed_vector()
{	
	vector3f velocity(3,0,0);
	if((_gps.have_gps() == false) || _gps.status() < AP_GPS::GPS_FIX_TYPE_2D || _gps.num_sats() < AP_GPS::GPS_MIN_SATS)
	{
		float airspeed = 3;							     // Here we assume airspeed is more valid and we will use this to estimate groundspeed|velocity for dead reckoning navigation
		if(_airspeed->enabled())
		{
			airspeed = _airspeed->get_airspeed();    	 // we have a valid airspeed reading from a sensor then we use this airspeed value
		}
		else
		{
			airspeed = _last_airspeed;			     	// if we dont have a valid airspeed sensor reading we will use the value of _last_airspeed which is updated once we have a valid GPS reception as our airspeed
		}		
		velocity     = dcmMatrix.colx() * airspeed;  	// airspeed in earth frame	
		velocity     = velocity + wind;			     	// estimated ground velocity
	}
	else
	{
		velocity = _gps.velocity();					    // Here we assume the velocity is far more accurate than airspeed and we use the velocity to update last airspeed for later on when we dont have GPS reception
	}

	return Vector2f(velocity.x, velocity.y);
}


void  AP_AHRS::setHil()
{
	_hil_mode = true;
	
	_last_hil_millis = millis();
}

bool  AP_AHRS::isflying(){
	if(groundspeed_vector().length() >= 3.5 && 
	   airspeed_estimate() >= 3.5 		    && 
	   altitude_estimate() >= 6)
	{		
		return true;
	}
	else
	{
		return false;
	}
}





