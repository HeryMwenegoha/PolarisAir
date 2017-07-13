 #include "AP_TECS.h"
 #include "Vectors.h"
 void AP_TECS::update_params()
{   
   _maxClimbRate     = 5.0f;
   _minSinkRate      = 2.0f;
   _timeConst        = 5.0f;
   _thrDamp          = 0.5f;
   _integGain        = 0.1f;
   
   /* Vertical acceleration limit (m/s/s)
	* 
	*/
   _vertAccLim       = 7.0f;
   
   /* Height Complimentary Filter Frequency (radians/second)
    * Cross over frequency of the complementary filter to fuse baro data and vertical acceleration data
	* Should be between 1.0 - 5.0 increased in steps of 0.05
    */
   _hgtCompFiltOmega = 1.0f; 
   
   /* Speed Complimentary filter (radians/second)
    * Crossover frequency of the filter used to fuse airspeed and longitudinal acceleration to get a better estimate of speed
	* range 0.5 - 2.0
	* increment 0.5
	*/
   _spdCompFiltOmega = 2.0f;
   
   /* Bank Angle Compesation gain
    * range 5.0 - 30.01f
	* Increment 1.0
	*/
   _rollComp         = 10.0f;
   
   /* TECS speed priority definition
    * 1.0 - Equal Priority to height and speed errors
	* 2.0 - Prioritise speed errors
	* 0.0 - Prioritise height errors
    * Range 0.0 - 2.01f
	* Increment 0.1
    */
   _spdWeight        = 1.0f;
   
   /* TECS Controller Pitch Damping Gain
    * Damping Gain for the Pitch demand loop if Increased it will add damping and correct for oscillations in speed and height
	* Range 0.1 - 1.0
	* Increment 0.1
	*/
   _ptchDamp         = 0.25f;
   
   /* Maximum Descent Rate in m/s/s
	* The maximum descent rate the controller will use. Should be set to the value the aircraft can achieve at pitch angle limit
	* Increment 0.1-alpha
	* Range 0.0 - 20.0
	*/
   _maxSinkRate      = 5.0f;
   
   /* Airspeed during landing (m/s)
    * Used as goal airspeed when performing autonomous landing
	* Not useful if platform does not have an airspeed sensor
    */
	
	/* Cruise throttle during landing
	 * If no airspeed is present then this paramater is used instead
	 * Range -1 - 100
	 * Incremnet 0.1
	 * -1 Not used and Landairspeed is used instead
	 */
   _landTrottle      = (float)PARAMETERS->TECS_thr_land;
   
   /* Weight applied to speed control during landing
	* has the same effect as spdweight with the difference being that this is applied during landing 
	* Range 0.0 - 2.0
	* 2.0 - Prioritise speed and ignore height error [Possible to overshoot]
	* 0.0 - Prioritise height and ignore speed error [Possible Stall]
	* 1.0 - Equal Priorities in speed and Height errors
	* Increment 0.1
    */
   _spdWeightLand    = 1.0f;
   
   _pitch_max        = (float)PARAMETERS->AUTO_pitchmax; // deg
   _pitch_min        = (float)PARAMETERS->AUTO_pitchmin; // deg
   _land_sink        = 0.25f;
   
   /* Land Controller Time Constant (sec)
    * TECS controller time constant whn in landing stages. Generally smaller than general timeconstant to allow for faster flare response
	* range 1.0 - 5.01f
	* increment 0.2
	*/
   _landTimeConst    = 2.0f;
   
   /* TECS controller sink rate to pitch gain during FLARE
	* sink rate gain for the pitch demand loop when in the final landing stage of flight. Larger than TECS_PTCH_DAMP to allow for better sink rate control during flare
	* Range 0.1 - 1.01f
	* Increment 0.1
	*/
   _landDamp         = 0.5f;
   
   /* Maximum pitch during landing (deg)
    * Range -5 - 40
	* Increment 1
	*
	*/
   _land_pitch_max   = 10.0f;
}

void  
AP_TECS::update_50Hz(float alt, float climbrate)
{
   /*@ We want to:
	*@ 1. Update the gains based on Advanced User settings on calling class loop speed :: 50Hz
	*@ 2. Make Sure that we are running at 50Hz
	*@ 3. implement 3order comp filter to incoming height to obtain:
	*@ 3.1. tecs_height        _integ3_state
	*@ 3.2. tecs_climb_rate    _climb_rate
	*@ These will be used to estimate the current and demanded energy levels
	*/
	update_params();
	
	// Get altitude estimate
    float hgt_afe = _ahrs.altitude_estimate();

	uint64_t tnow = micros();
	
	float DT      = (tnow -  _update_50Hz_last_usec) * 1.0e-6f;
  
	// Taking too long reset stuff here
	if(DT > 1.0f)
	{
	  //_last_vel_dot    = 0;
	  _vel_dot         = -gravity * sin(_ahrs.pitch) + _ahrs.acc.x; 
	  _integ3_state    = hgt_afe;
	  _climb_rate      = 0.0f;
	  _integ1_state    = 0.0f;
	  DT               = 0.02f; // 50Hz
	}   	
	_update_50Hz_last_usec = tnow;
	
	vector3f accel_bf  = vector3f(_ahrs.acc.x, _ahrs.acc.y, _ahrs.acc.z);	// body frame accelerations
	vector3f accel_ef  = _ahrs.dcm() * accel_bf;   							// accelerations in the earth frame
	float hgt_ddot_mea = -(accel_ef.z + gravity); 			        		// NED is positive so to make Upward Movement positive we add (-)

	// Filter calculations
	float omega2       = _hgtCompFiltOmega * _hgtCompFiltOmega;
	float hgt_err      = hgt_afe - _integ3_state;
	float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;
	_integ1_state      = _integ1_state + integ1_input * DT;

	float integ2_input = hgt_ddot_mea + _integ1_state + hgt_err * omega2 * 3.0f;
	_climb_rate        = _climb_rate + integ2_input * DT;
	
	_climb_rate        = _climb_rate * 0.05f + 0.95f * climbrate;

	float integ3_input = _climb_rate + hgt_err * _hgtCompFiltOmega * 3.0f;
	
	// There's no need for this but just incase you know..
	if(DT > 1.0f)
	{
	  _integ3_state = hgt_afe;
	}
	else
	{
	   _integ3_state = _integ3_state + integ3_input* DT; 
	   
	   _integ3_state = _integ3_state * 0.05f + alt * 0.95f;
	}

	// low pass filter on x acceleration
	float temp    = -gravity * sin(_ahrs.pitch) + _ahrs.acc.x;
	float FC      = 10; // very small changes
	float RC      = 1/ (2 * PI * FC);
	float alpha   = DT/(RC + DT);
	_vel_dot      = alpha * temp + (1-alpha) * _last_vel_dot;
	_last_vel_dot = _vel_dot;
}


void 
AP_TECS::update_speed()
{
    /*@ Speed Estimates
     *@ Stall prevention
     *@ Speed Checks
     *@ update TASmin and TASmax
     *@ Get Airspeed from airspeed indicator, if not available estimate min max
     *@ Second Order Complimentary Filter
     */
	 
	 uint64_t tnow           = micros();
	 float DT                = (tnow - _update_speed_last_usec) * 1.0e-6f;
	 _update_speed_last_usec = tnow;
	 
     float EAS2TAS = 1; // Our flights for now are below 2000m so we can safely assum TAS == EAS == IAS
     _TAS_dem      = _EAS_dem * EAS2TAS;
     _TASmax       = (float)PARAMETERS->max_speed * EAS2TAS;
     _TASmin       = (float)PARAMETERS->min_speed * EAS2TAS;
	 float div     = cos(_ahrs.roll);
	 div           = constrain_float(div, 0.1f, 1.0f);
     if(PARAMETERS->TECS_stallPrevent == 1)
     {
	   // raise the minimum airspeed to prevent aircraft from stalling
       float _load_factor = 1/div;
       _TASmin            = _TASmin * sqrt(_load_factor);
     }
	 
	 if(_TASmax < _TASmin){
	    _TASmax = _TASmin;
	 }
	 
	 if(_TASmin > _TAS_dem){
	    _TASmin = _TAS_dem;
	 }
	 
     if(DT > 1.0f)
     {
       _integ5_state = _EAS * EAS2TAS; // to current speed
       _integ4_state = 0.0f;
       DT            = 0.1f;  
     }
	 	 
	 // airspeed estimate
	 _EAS = _ahrs.airspeed_estimate();
	 if(_EAS == 0){
		_EAS = 0.5f * (float)(PARAMETERS->min_speed  +  PARAMETERS->max_speed);       
	 }
	_EAS = constrain_float(_EAS, 3.0, 40);


     // second order complimentary fiter for speed::	 
     float arspdErr      = (_EAS * EAS2TAS) - _integ5_state;
     float integ4_input  = arspdErr * _spdCompFiltOmega * _spdCompFiltOmega;
     if(_integ5_state < 3.1f)
     {
       integ4_input = fmax(integ4_input, 0.0f);
     }
     _integ4_state  = _integ4_state + integ4_input * DT;
	 
	
     float integ5_input =  _integ4_state + _vel_dot + arspdErr * _spdCompFiltOmega * 1.4142f;
     _integ5_state      = _integ5_state + integ5_input * DT;     
     _integ5_state      = max(_integ5_state, 3.0f);
}


void 
AP_TECS::_update_demanded_speed(void)
{
    /*@ Speed demand Processig
     *@ We are now ready to process the demanded speed
     *@ Since we have the EASdem, we want to get _TASdem_adjusted for the rate limits 
     *@ we want to limit the rate of change of speed and whatnots based on STEdotmax and min
     *@ what we will use for calculations is _TASdem_adj and therefore check difference between adjusted and direct demand
     */ 
     if(_badDescent || _underspeed)
     {
       _TAS_dem = _TASmin;
     }
	 
     _TAS_dem = constrain_float(_TAS_dem, _TASmin, _TASmax);
     float velRatemax;
     float velRatemin;     
     velRatemax = 0.5f * _STEdot_max/_integ5_state;
     velRatemin = 0.5f * _STEdot_min/_integ5_state; 
     if((_TAS_dem - _TAS_dem_adj) > (0.1 * velRatemax))
     {
       _TAS_dem_adj   =  _TAS_dem_adj + 0.1f * velRatemax; // increase the smoothed speed over twice the time constant :: 5Hz
       _TAS_rate_dem  = velRatemax;
     }
     else if((_TAS_dem - _TAS_dem_adj) < (0.1 * velRatemin))
     {
       _TAS_dem_adj   =  _TAS_dem_adj + 0.1f * velRatemin; // reduce the smoothed speed
       _TAS_rate_dem  = velRatemin;      
     }
     else
     {
       _TAS_dem_adj   =  _TAS_dem;
       _TAS_rate_dem  = (_TAS_dem_adj - _TAS_dem_adj_last)/0.1f;
     }    
     _TAS_dem_adj      = constrain_float(_TAS_dem_adj, _TASmin, _TASmax);
     _TAS_dem_adj_last = _TAS_dem;
}

void
AP_TECS::_update_demanded_height(void)
{
  // 2 point moving average to make demanded height be updated at 5Hz instead of 10Hz
  _hgt_dem        = 0.5f * (_hgt_dem + _hgt_dem_in_old);
  _hgt_dem_in_old = _hgt_dem;
  
  if((_hgt_dem - _hgt_dem_prev) > (0.1f * _maxClimbRate))
  {
      _hgt_dem = _hgt_dem_prev + 0.1f * _maxClimbRate;
  }
  else if((_hgt_dem - _hgt_dem_prev) < (-0.1f * _maxSinkRate))
  {
       _hgt_dem   = _hgt_dem_prev - 0.1f * _maxSinkRate; 
  }
  _hgt_dem_prev = _hgt_dem;
   
   // First Order Lag to get smoother height_demanded
  _hgt_dem_adj      = 0.05f * _hgt_dem + 0.95f * _hgt_dem_adj_last;
  
  
  // Landing::height_rate_dem => sink_rate..
  float aim_height = 12;
  if(_flight_stage_global == FLIGHT_LAND)// || (_flight_stage_global == FLIGHT_LAND_APPROACH && _integ3_state <= aim_height))
  {
	_integ7_state = 0;
	if(_flare_counter == 0)
	{
		_hgt_rate_dem = _climb_rate;
		_land_hgt_dem = _hgt_dem_adj;
	}
	
	// increase the demanded height rate over 1second to prevent overshoots.
	if(_flare_counter < 10)
	{
		_hgt_rate_dem = _hgt_rate_dem * 0.8f - 0.2f	*	_land_sink;
		_flare_counter++;
	}
	else
	{
		_hgt_rate_dem = -_land_sink;
	}
	
	// i added this but we can test what happens when removed
	/*
	if(_integ3_state <= 5)
	{
		_hgt_rate_dem = -_land_sink;
	}
	*/
	
	_land_hgt_dem += 0.1f * _hgt_rate_dem; // continue taking the land hgt demanded below the surface
	_hgt_dem_adj   = _land_hgt_dem;
  }
  else if(_flight_stage_global == FLIGHT_LAND_APPROACH)
  {
	_hgt_rate_dem  = 1.0;
	_flare_counter = 0;    
  }
  else
  {
	_hgt_rate_dem  = (_hgt_dem_adj - _hgt_dem_adj_last)/0.1f;
	_flare_counter = 0;  
  }
  
  float new_hgt_dem = _hgt_dem_adj;
  if(_flight_stage_global == FLIGHT_LAND)// || _flight_stage_global == FLIGHT_LAND_APPROACH) //--> modified
  {
	new_hgt_dem += (_hgt_dem_adj - _hgt_dem_adj_last) * 10.0f *(_timeConst + 1);
  }
  
  _hgt_dem_adj_last = _hgt_dem_adj; 
  _hgt_dem_adj = new_hgt_dem;
    
}


void 
AP_TECS::_update_energies(void)
{
   // demanded energies::
   _SPE_dem = _hgt_dem_adj * gravity;
   _SKE_dem = _TAS_dem_adj * _TAS_dem_adj * 0.5f;
       
   _SPEdot_dem = _hgt_rate_dem * gravity;
   _SKEdot_dem = _TAS_rate_dem * _integ5_state; // this please
	   
   // current estimated energies::  
   _SPE_est = _integ3_state * gravity;
   _SKE_est = _integ5_state * _integ5_state * 0.5f;
       
   _SPEdot = _climb_rate * gravity;
   _SKEdot = _vel_dot    * _integ5_state; 
}


void AP_TECS::_update_throttle_dem_no_arspd()
{
    // linear interpolation based demanded throttle..
    float normThr;
    float pitchSlope;           // || _flight_stage_global == FLIGHT_LAND_APPROACH  
	float thr_land = (float)PARAMETERS->TECS_thr_land;
		
	
	if(_flight_stage_global == FLIGHT_LAND 					&& thr_land >= 0)
	{
	  normThr = thr_land * 0.01f;
	}
	else if(_flight_stage_global == FLIGHT_LAND_APPROACH	&&	thr_land >= 0)
	{
	  normThr = thr_land * 0.01f;
	}
	else
	{
	  normThr = (float)PARAMETERS->cru_thr * 0.01f;
	}
         
    if(_pitch_dem > 0.0f && _PITCHmaxf > 0.0f)
    {
      pitchSlope    = _pitch_dem/_PITCHmaxf;
      _throttle_dem = pitchSlope * (_THRmaxf - normThr)   + normThr;   
    }
    else if(_pitch_dem < 0.0f && _PITCHminf < 0.0f)
    {
      pitchSlope    = _pitch_dem/_PITCHminf;
      _throttle_dem = pitchSlope * (_THRminf - normThr)   + normThr; 
    }
    else
    {
      _throttle_dem = normThr;
    }
	   
	const matrix3f dcm  = _ahrs.dcm();
	float cosPhi     = sqrtf((dcm.a.y * dcm.a.y) + (dcm.b.y*dcm.b.y));
    //float cosPhi   = cos(_ahrs.roll);
    cosPhi           = constrain_float(cosPhi * cosPhi, 0.1f, 1.0f);
	
	// Feedforward rate loss is demanded 
    float STEdot_dem = _rollComp * (1.0f/cosPhi - 1); // _rollComp is 10 times the maxSinkRate in a 45degree  bank     
    _throttle_dem   = _throttle_dem + STEdot_dem/(_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);	      
}

void
AP_TECS::_update_throttle_dem_arspd()
{
	// The maximum allowable specific potential energy error i.e. the difference between what is demanded and what has been estimated is governed by
	// the difference between maximum specific kinetic energy and the demanded specific kinetic energy 
	float SPE_err_max = 0.5f * _TASmax * _TASmax - _SKE_dem;
	float SPE_err_min = 0.5f * _TASmin * _TASmin - _SKE_dem;
	
	_STE_error 		   = constrain_float((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
	float STEdot_dem   = constrain_float((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
	float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;
	
	// second order filter to STEdot_error -> removes accelerometer noise from measurement
	STEdot_error = 0.2f * STEdot_error + 0.8f*_STEdotErrLast;
	_STEdotErrLast = STEdot_error;
	
	if(_underspeed)
	{
		_throttle_dem = 1.0f;
	}
	else
	{
		// Converts Specific energy to Throttle
		float K_STE2Thr = 1/(_timeConst * (_STEdot_max - _STEdot_min));
		
		float ff_throttle = 0;
		float nomThr 	  = (float)PARAMETERS->cru_thr * 0.01f; // cruise throttle
		float cosPhi 	  = cos(_ahrs.roll);
		float axb    = cosPhi * cosPhi;
		cosPhi       = constrain_float(axb, 0.1f, 1.0f);
		
		// Adds a Roll Compensation term to the induced drag generated in turns to give a feed forward term to throttle
		STEdot_dem  = STEdot_dem	+	_rollComp * (1.0f/cosPhi - 1); 					// _rollComp is 10 times the maxSinkRate in a 45degree  bank
		ff_throttle = nomThr + STEdot_dem/(_STEdot_max - _STEdot_min)	*	(_THRmaxf - _THRminf);
		
		// PD + FF Throttle
		_throttle_dem = (_STE_error + STEdot_error * _thrDamp)* K_STE2Thr + ff_throttle;
		
		// Constrain throttle demand
		_throttle_dem = constrain_float(_throttle_dem, _THRminf, _THRmaxf);
		
		// Rate Limit the PD + FF throttle
		
		
		// Integrator Upper and Lower Limits
		float maxAmp    = 0.5f * (_THRmaxf - _THRminf);
		float integ_max = constrain_float((_THRmaxf - _throttle_dem + 0.1f), -maxAmp, maxAmp);
		float integ_min = constrain_float((_THRminf - _throttle_dem - 0.1f), -maxAmp, maxAmp);
		
		// Throttle Integrator::
		_integ6_state = _integ6_state  + (_STE_error * _integGain) * _DT * K_STE2Thr;
		if(_flight_stage_global == FLIGHT_TAKEOFF)
		{
			_integ6_state = integ_max;
		}
		else
		{
			_integ6_state = constrain_float(_integ6_state, integ_min, integ_max);
		}
		
		// sum integral + PD + FF
		_throttle_dem = _throttle_dem + _integ6_state;	
	}
	_throttle_dem = constrain_float(_throttle_dem, _THRminf, _THRmaxf);		
}


void 
AP_TECS::_update_pitch_dem(void)
{
	float SKE_weighting = constrain_float(_spdWeight, 0.0f, 2.0f);
	if(_ahrs.airspeed().enabled() == false)
	{
	  SKE_weighting = 0.0f;
	}
	else if(_underspeed || _flight_stage_global == FLIGHT_TAKEOFF) 
	{
	  SKE_weighting = 2.0f;	
	}	
	
	if(_flight_stage_global == FLIGHT_LAND || _flight_stage_global == FLIGHT_LAND_APPROACH)
	{
		PARAMETERS->arspdEnabled == 1 ? SKE_weighting = constrain_float(_spdWeightLand, 0.0f, 2.0f) : SKE_weighting = 0;
	}
	// Serial.print(_underspeed);
	// Serial.print("	");
	// Serial.print(_integ5_state);
	// Serial.print("	");
	// Serial.println(SKE_weighting);
	
	float SPE_weighting = 2.0f - SKE_weighting;   

    // Calculate the energy balance errors	
	float SEB_dem       = _SPE_dem * SPE_weighting - _SKE_dem * SKE_weighting;
	float SEBdot_dem    = _SPEdot_dem * SPE_weighting - _SKEdot_dem * SKE_weighting;
	float SEB_error     = SEB_dem - (_SPE_est * SPE_weighting - _SKE_est * SKE_weighting);
	float SEBdot_error  = SEBdot_dem - (_SPEdot * SPE_weighting - _SKEdot * SKE_weighting);
	
	// Integral Part
	float integ7_input = SEB_error * _integGain; 
	if(_pitch_dem > _PITCHmaxf){
	  integ7_input = fmin(integ7_input, _PITCHmaxf - _pitch_dem);
	}
	else if(_pitch_dem < _PITCHminf){
	  integ7_input = fmax(integ7_input, _PITCHminf - _pitch_dem);
	}
	float _integ7_delta = integ7_input * _DT;
	
	
	// Gain converts angle in radians to specific energy:
	float gainInv = _integ5_state * _timeConst * gravity; // Theta * Gain = Specific Energy
	float temp    = SEB_error + SEBdot_dem * _timeConst;  // Proportional_Term  + FF_term
	if(_flight_stage_global == FLIGHT_LAND)
	{
		temp +=	SEBdot_error * _landDamp; 
	}
	else
	{
		temp +=	SEBdot_error * _ptchDamp;                 // (Proportional_Term + FF_Term) + Derivative Term
	}
	
	if(_flight_stage_global == FLIGHT_TAKEOFF)
	{
	  temp +=_PITCHminf * gainInv; // if we are taking off cause the pitch to be down albeit by lowering the energy we want
	}
	
	// constrain the integral energies to +-5degrees of limits to allow 5degrees variation around maximum values
	// therefore is temp is 0, integral can cause 
	float _integ_min   = (gainInv * (_PITCHminf - 0.0783f)) - temp;
	float _integ_max   = (gainInv * (_PITCHmaxf + 0.0783f)) - temp;
	float _integ7_range = _integ_max - _integ_min;
	
	_integ7_delta = constrain_float(_integ7_delta, -_integ7_range*0.1f, _integ7_range*0.1f);
	
    _integ7_state = _integ7_state + _integ7_delta; // Integral Term

	_integ7_state = constrain_float(_integ7_state,_integ_min , _integ_max);
	
	
	// Add the two loops together
	// 1. (Proportional_Term + FF_Term) + Derivative Term
	// 2. Integral_Term
	// 3. (Proportional_Term + FF_Term) + Derivative Term = Temp + Integral_Term
	_pitch_dem_unc = (temp + _integ7_state)/gainInv;
	
	_pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);
	
	float ptchRateIncr = _DT * _vertAccLim / _integ5_state;
	if((_pitch_dem - _last_pitch_dem) > ptchRateIncr)
	{
	  _pitch_dem = _last_pitch_dem + ptchRateIncr;
	}
	else if((_pitch_dem - _last_pitch_dem) < -ptchRateIncr)
	{
	  _pitch_dem = _last_pitch_dem - ptchRateIncr;
	}
	_pitch_dem      = constrain_float(_pitch_dem, _PITCHminf, _PITCHmaxf);
	_last_pitch_dem = _pitch_dem;
}


void 
AP_TECS::update_pitch_throttle(float hgt_dem, float EAS_dem, uint16_t _flight_stage)
{  
  uint64_t tnow = micros();
  _DT           =(tnow - _update_pitch_throttle_last_usec)*1e-6f;
  if(_DT       >= 0.1f)
  {
    _update_pitch_throttle_last_usec = tnow;

	
    /*@ After this we will be able to process the current kinectic energy
	 *@ And the current rate of change of kinectic Energy. not the demanded
	 *@ process speed update including demanded TAS based on previous 
	*/
    update_speed();
	
    // update demanded variables::
    _EAS_dem = EAS_dem;
    _hgt_dem = hgt_dem;
    
	
	// Adjust Time Constant based on flight stage and limiter
    if(_flight_stage_global == FLIGHT_LAND || _flight_stage_global == FLIGHT_LAND_APPROACH)
	{
		_timeConst = _landTimeConst;
		if(_timeConst < 0.1f)
		{
			_timeConst = 0.1f;
		}
	}
	else
	{
		_timeConst = 5.0f;
	}
	
         
    /*@ update the maximum and minimum throttle
     *@ later:
     *@ 1. check the flight stage
     *@ 2. update the throttle based on the flight stage
     *@ 3. if flight stage is take-off then put full throttle until we are told that we are done with stage
     */
    if(_flight_stage_global == FLIGHT_TAKEOFF)
    {
		_THRmaxf = (float)PARAMETERS->TECS_thr_tomax * 0.01f;
    }
    else
    {
		_THRmaxf = (float)PARAMETERS->max_thr_perc   * 0.01f;
    }
    _THRminf   	 = (float)PARAMETERS->min_thr_perc   * 0.01f;
   
    /*@ update max and min pitch
     *@ later:
     *@ 1. check the flight stage
     *@ 2. pitch max and min needs to correspond to the respective flight stage
     *@ For now AUTOPITCHMAX AND MIN will do
     */ 
    _pitch_max     =  (float)PARAMETERS->AUTO_pitchmax;   
	if(_pitch_max <= 0){
	   _pitch_max  =  (float)PARAMETERS->max_pitch_deg;
	}
	
	_pitch_min     =  (float)PARAMETERS->AUTO_pitchmin;
	if(_pitch_min >= 0){
	  _pitch_min   =  (float)-PARAMETERS->max_pitch_deg;
	}		
	// final landing stage. i.e. +- 10 degrees -> +-15degrees with saturation
	if(_flight_stage_global == FLIGHT_LAND) // Final landing stage
	{
	  /*
	  _pitch_min   = max(_pitch_min, PARAMETERS->TECS_land_pitchmin);  // deg
	  
	  if(PARAMETERS->TECS_land_pitchmax > 0)
	  {
	  	_pitch_max = min(_pitch_max, PARAMETERS->TECS_land_pitchmax);  // deg
	  }
	  _THRminf = 0.0f;
	  */
	}
	else if(_flight_stage_global == FLIGHT_LAND	&&	(-_climb_rate)	> _land_sink) 
	{	

		if(PARAMETERS->TECS_land_pitchmax > 0)
		{
			_pitch_max = min(_pitch_max, (float)PARAMETERS->TECS_land_pitchmax);  // deg
		}
		_THRminf = 0.0f;
	  
		float flare_secs    = 5;
		float time_to_flare = (-_ahrs.altitude_estimate()/_climb_rate) - flare_secs;
		if(time_to_flare < 0)
		{
			// we should have started flaring right now.. 
			// do this if not yet in  the landing stage..
			_pitch_min = max(_pitch_min, (float)PARAMETERS->TECS_land_pitchmin);

		    _flaring = true;
		}
		else if(time_to_flare < _timeConst * 2)
		{
			_flaring = false;
			// move from stab/auto min pitch to land min pitch::at twice the time constant
			float p = time_to_flare / (_timeConst * 2);
			float p_stab = 0;
			if(PARAMETERS->AUTO_pitchmin < 0)
			{
				p_stab = (float)PARAMETERS->AUTO_pitchmin;
			}
			else if(PARAMETERS->AUTO_pitchmin >= 0)
			{
				p_stab = (float)-PARAMETERS->max_pitch_deg;
			}
			float pitch_limit = p_stab	*	p	+	(1-p)	*	(float)PARAMETERS->TECS_land_pitchmin;
			
			_pitch_min = max(_pitch_min, pitch_limit);
		}
	}
		    
    
    /*@ Convert stuff to radins
     *@ Mostly the angles.
     */
    _PITCHmaxf = ToRad(_pitch_max);
    _PITCHminf = ToRad(_pitch_min);
	_flight_stage_global = _flight_stage;

    
    
    /*@ Initialise states if DT is greater than 1second
     *@ Good reset mechanism
     *@ Needs also to initialise variables for Take-off stage
     */
    if(_DT > 1.0f)
    {
      _integ6_state      = 0.0f;
      _integ7_state      = 0.0f;
      _throttle_dem      = (float)PARAMETERS->cru_thr * 0.01f; 
      _last_throttle_dem = (float)PARAMETERS->cru_thr * 0.01f;
      _last_pitch_dem    =  _ahrs.pitch;
      _hgt_dem_adj_last  =  _ahrs.altitude_estimate();
      _hgt_dem_adj       =  _hgt_dem_adj_last;
      _hgt_dem_prev      =  _hgt_dem_adj_last;
      _hgt_dem_in_old    =  _hgt_dem_adj_last;
      //_TAS_dem_last      = _TAS_dem; // using _TAS_dem_adj_last is used instead of this
      _TAS_dem_adj       = _TAS_dem;
      _TAS_dem_adj_last  = _TAS_dem;
      _underspeed        = false;
      _badDescent        = false;
      _DT                = 0.1f;      
    }
    else if(_flight_stage_global == FLIGHT_TAKEOFF)
    {
      if(_ahrs.airspeed().enabled() == true)
	  {
       _PITCHminf  = ToRad(-5);             // minimum pitch during take-off
	  }
      else
	  {
        _PITCHminf = ToRad(-2);
        _PITCHmaxf = ToRad(7.5);
      }
      _hgt_dem_adj_last = _ahrs.altitude_estimate();   // current raw altitude (not filtered)
      _hgt_dem_adj      = _hgt_dem_adj_last;
      _hgt_dem_prev     = _hgt_dem_adj_last;
      _TAS_dem_adj_last = _TAS_dem;
      _TAS_dem_adj      = _TAS_dem;
      _underspeed       = false;
      _badDescent       = false;
    }	
    
    
    /*@ Specific Total Energy and Energy Rates Calculations Limits
     *@ STE = hmax * gravity
     */
     _STEdot_max = _maxClimbRate * gravity;
     _STEdot_min = -_minSinkRate * gravity;
     
	 
	 /*@ Updating the demanded speed
	  *@ You can think of this as filtering what the user wants
	  *@ To get:
	  *@ 1. Adjusted|Filtered|Smoothed Demanded Airspeed
	  *@ 2. Filtered rate of change of speed
	  *@ After this we will be able to estimate the demanded kinectic energy
	  */
	 _update_demanded_speed();
     
     
     /*@ Process the demanded height to get the demanded-adjusted-height hgt_dem_adj
      *@ We will limit demanded-hgt-adj to the climb rates:
      *@ Limit the height change as we did to the velocity change
      *@ First order lag filter to the adjusted height
      *@ Use a 5Hz update on the speed
      */
	  _update_demanded_height();
	  
     
      
      /*@ DETECT UNDERSPEED
       *@ Conditions:
       *@ 1. _integ5_state is less than 95% of _TASmin
       *@ 2. demanded throttle is greater than 96% of max throttle
       *@ 3. you are not on final stage **** Needs implementing
       *@ or
       *@ 4. _integ3_state is less than the required|demanded hgt and our previous condition was underspeed
       *@ last condition is to keep the underspeed on
       */
	   
	   // If we have been above min speed for the past 3 seconds but in underspeed condition then say that underspeed has been averted
	   if(_underspeed && (_integ5_state >= _TASmin * 1.15f) && (millis() - _underspeed_started_ms > 3000)){
		   _underspeed = false;
	   }
	   
       if(((_integ5_state < 0.90f * _TASmin) &&	(_throttle_dem >= 0.95f * _THRmaxf)   &&  (_flight_stage_global != FLIGHT_LAND)) || ((_integ3_state < _hgt_dem_adj)  && _underspeed))
       {
           _underspeed = true;
		   
		   if(_integ5_state < _TASmin * 0.90f){
			   _underspeed_started_ms = millis(); // reset the time that i have been in underspeed condition
		   }
       }
       else
       {
           _underspeed = false;
       }
         		 
		 
      /*@ Update the energies available and demanded
	   *@ SKE and SKEdot demand
	   *@ SKE and SKEdot current
	   *@ SPE and SPEdot demand
	   *@ SPE and SPEdot current
       */
	   _update_energies();
      
      
      /*@ Update the demanded throttle  between 0 and 1:
       *@ Can be throttle with airspeed or without
       *@ First implement Non-Airspeed throttle
       *@ Linear interpolation between demanded pitch to throttle
       *@ Plus feedforward gain resulting from induced drag calculation 
       */
       if(_ahrs.airspeed().enabled() == true)
       {
	     _update_throttle_dem_arspd();   // no use of synthetic airspeed
       } 
       else
       {
	     _update_throttle_dem_no_arspd();
       }
       
       /*@ Detect a bad descent here
        *@ Check the following conditions
        *@ Blank for now::
        *@ 1. _inte I need to implement this..
        */
		/*
		float SPE_err_max = 0.5f * _TASmax * _TASmax - _SKE_dem; // allowable error in Potential energy 
		float SPE_err_min = 0.5f * _TASmin * _TASmin - _SKE_dem;		
		float _STE_error = constrain_float((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
		*/
		
		float _STEdot = _SKEdot + _SPEdot;
		if(((!_underspeed	&& (_STE_error > 200)) &&
		   (_STEdot < 0.0f)	&&
		   (_throttle_dem >= 0.95*_THRmaxf)) ||
		   (_badDescent && !_underspeed && (_STE_error > 0.0f)))
		   {
			_badDescent = true;
		   }
		   else
		   {
			_badDescent = false;
		   }
       
       
       /*@ Update Pitch demanded
        *@ Returns pitch demand in radins between pitchmaxf and pitchminf in radians
        *@ Uses specific energy balance to generate demanded pitch:
        *@ On Takeoff SKE weighting is important that SPE
        */
		_update_pitch_dem();
  }
}
