#ifndef Vector2_h
#define Vector2_h
#include "Arduino.h"
template <typename T>
struct Vector2
{
  public:
	T x ,y;

	// construct
	Vector2<T>(){
		x = y = 0;
	}

	// constructor 2
	Vector2<T>(T xin, T yin){
		x = xin;
		y = yin;  
	}
  
	bool is_nan(void)const
	{
		return (isnan(x) || isnan(y));
	}
	

	bool is_inf(void)const
	{
		return (isinf(x) || isinf(y));
	}
	

	bool is_zero(void)const
	{
		return (x==0 && y==0);
	}
	
  
	float length();

	void normalise();
	
	Vector2<T> normalized();

	Vector2<T> operator * (const T v) const;

	T operator * (const Vector2<T> v)const;

	T operator % (const Vector2<T> v)const;

	Vector2<T> operator - (void) const;
};

typedef Vector2<float>    Vector2f;
typedef Vector2<int16_t>  Vector2i;
#endif