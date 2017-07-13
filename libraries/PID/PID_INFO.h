#ifndef PID_INFO_h
#define PID_INFO_h

#define ROLLPID	 0
#define PITCHPID 1
#define YAWPID   2

struct pid_info
{
	float P;
	float I;
	float D;
};

struct _gains
{	
	float 	*ki;
	float 	*kp;
	float 	*kd;
	float 	*tau;
	uint8_t *rmax;
	uint8_t *imax;
};

struct _Pgains
{	
	float 	*ki;
	float 	*kp;
	float 	*kd;
	float 	*tau;
	uint8_t *rmax;
	uint8_t *imax;
	float 	*roll_ff;	
};

struct _Ygains
{	
	float 	*ki;
	float 	*kp;
	float 	*kd;
	float 	*tau;
	uint8_t *rmax;
	uint8_t *imax;
	float	*roll_ff;	
};
#endif