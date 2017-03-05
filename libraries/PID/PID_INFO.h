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
	float *ki;
	float *kp;
	float *kd;
	float *tau;
	float *rmax;
	float *imax;
	// float *max_aux;	
	// float *min_aux;	
};

struct _Pgains
{	
	float *ki;
	float *kp;
	float *kd;
	float *tau;
	float *rmax;
	float *imax;
	float *roll_ff;
	float *max_aux;	
	float *min_aux;	
};

struct _Ygains
{	
	float *ki;
	float *kp;
	float *kd;
	float *tau;
	float *rmax;
	float *imax;
	float *roll_ff;
	float *max_aux;	
	float *min_aux;	
};
#endif