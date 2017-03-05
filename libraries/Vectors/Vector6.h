#ifndef Vector6_h
#define Vector6_h
#include "Arduino.h"
#include "inttypes.h"
template <typename T>
class vector6
{
	public:
	T x1, x2, x3, x4, x5, x6;
	vector6<T>(){
		x1 = x2 = x3 = x4 = x5 = x6 = 0;
	}
	
	vector6<T>(T x1_,	T x2_,	T x3_,	T x4_,	T x5_,	T x6_){
		x1 = x1_;
		x2 = x2_;
		x3 = x3_;
		x4 = x4_;
		x5 = x5_;
		x6 = x6_;
	}
	
	T magnitude(void);
	
	vector6<T> operator + (const vector6<T> v)const; // add
	vector6<T> operator - (const vector6<T> v)const; // subtract
	T operator * (const vector6<T> v)const; // dot product
	vector6<T> operator * (const T v)const; 		 // dot product
	
};

typedef vector6<float>	vector6f;
typedef vector6<int8_t>	vector6i;
#endif