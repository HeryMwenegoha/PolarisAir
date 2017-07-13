/*@ This is the TECSController Class
 *@ Total Energy Control System
 *@ Throttle Controls Total Energy
 *@ Pitch Controls the Energy balance
 *@ STE = SPE + SKE
 *@ Derived from Paul Risberough and NASA
 *@ Controller Creator: Hery A Mwenegoha
 */ 
 #pragma once
 #include "Arduino.h"
 #include "AP_Parameters.h"
 #include "AP_AHRS.h"
 class AP_TECS
 {
   public:
   AP_TECS(AP_AHRS &_ahrs_):
   _ahrs(_ahrs_)
   {
     PARAMETERS = &AP_params.ParameterStorage.list;
	 _update_50Hz_last_usec 		  = 0;
     _update_speed_last_usec 		  = 0;
	 _update_pitch_throttle_last_usec = 0;
	 _DT 							  = 0;
	 _STEdotErrLast 				  = 0;
	 gravity 						  = 9.81;
	 _flaring 						  = false;
   }
   
  /*
   * 50Hz calculations to estimate speeds and height and rates
   */   
   void  update_50Hz(float alt, float climbrate);
   
   /*@ Main Function that gives back the throttle demand and the pitch demand
    *@ Adopted from Paul Risberough
	*@ Implemented by Hery A Mwenegoha
    */
   void  update_pitch_throttle(float hgt_dem,  float EAS_dem, uint16_t _flight_stage);   
   float altitude(void){return _integ3_state;}  		 // above field elevation  
   float speed(void){return _integ5_state;}   		 // airspeed  
   float throttle_dem(void){return _throttle_dem;}   
   float pitch_dem(void)   {return _pitch_dem;}   
   float climbrate(void)  {return _climb_rate;}
   
   bool  flaring(){
	   return _flaring;
   }
   
    
   private:
   bool _flaring;
   
   uint32_t _underspeed_started_ms;
   
   // Times
   uint64_t _update_50Hz_last_usec;
   uint64_t _update_speed_last_usec;
   uint64_t _update_pitch_throttle_last_usec;
   float 	_DT; // main update diff
   
   AP_AHRS &_ahrs;
   struct _parameter_list_t *PARAMETERS;
   void  update_params();
   void  update_speed();
   void _update_demanded_speed(void);
   void _update_demanded_height(void);
   void _update_energies(void);
   void _update_throttle_dem_no_arspd();
   void _update_throttle_dem_arspd();
   void _update_pitch_dem(void);
   
   uint16_t _flight_stage_global;
   
   uint8_t  _flare_counter;  
   float gravity;
   float _integ3_state;
   float _integ5_state;
   float _integ1_state;
   float _EAS; // reading from arspdIndicator
   float _climb_rate;
   float _vel_dot;
   float _last_vel_dot;
   float _pitch_dem;
   float _throttle_dem;
   
   float _EAS_dem;
   float _TAS_dem;
   float _TASmax;
   float _TASmin;
   float _hgt_dem;
   float _THRminf;
   float _THRmaxf;
   float _PITCHmaxf;
   float _PITCHminf;
   float _integ4_state;
   float _integ6_state;
   float _integ7_state;
   float _last_throttle_dem;
   float _last_pitch_dem;
   float _pitch_dem_unc;
   float _hgt_dem_adj_last;
   float _hgt_dem_adj;
   float _hgt_dem_prev;
   float _hgt_dem_in_old;
   float _hgt_rate_dem;
   float _land_hgt_dem;
   //float _TAS_dem_last;
   float _TAS_dem_adj;
   float _TAS_dem_adj_last;
   float _TAS_rate_dem;
   boolean _underspeed;
   boolean _badDescent;
   float _STE_error;
   float _STEdotErrLast;
   float _STEdot_max;
   float _STEdot_min;
   float _SPE_dem;
   float _SKE_dem;
   float _SKEdot_dem;
   float _SPEdot_dem;
   float _SPE_est;
   float _SKE_est;
   float _SPEdot;
   float _SKEdot; 
   
   float _maxClimbRate;
   float _minSinkRate;
   float _timeConst;
   float _thrDamp;
   float _integGain;
   float _vertAccLim;
   float _hgtCompFiltOmega;
   float _spdCompFiltOmega;
   float _rollComp;
   float _spdWeight;
   float _ptchDamp;
   float _maxSinkRate;
   float _landTrottle;
   float _spdWeightLand;
   float _pitch_max;
   float _pitch_min;
   float _land_sink;
   float _landTimeConst;
   float _landDamp;
   float _land_pitch_max;
 };
