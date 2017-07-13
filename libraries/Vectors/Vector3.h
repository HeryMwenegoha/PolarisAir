#ifndef Vector3_h
#define Vector3_h
#include "inttypes.h"
/*@ Library implemented by Hery A Mwenegoha (C) 20/02/2016
 */
template <typename T>
struct vector3
{
	public:
	T x, y, z;
		
	vector3<T>()
	{
		x = y = z = 0;
	}
	
	vector3<T>(T a, T b, T c)
	{
		x = a;
		y = b;
		z = c;
	}
	
	void zero()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}
	
	T  magnitude(void)const
	{
		T x_ = x*x;
		T y_ = y*y;
		T z_ = z*z;
		return(sqrt(x_ + y_ + z_));
	}
	
	T  length(void)const
	{
		T x_ = x*x;
		T y_ = y*y;
		T z_ = z*z;
		return(sqrt(x_ + y_ + z_));
	}
	
	bool is_nan(void)const
	{
		return (isnan(x) || isnan(y) || isnan(z));
	}
	
	bool is_inf(void)const
	{
		return (isinf(x) || isinf(y) || isinf(z));
	}
	
	bool is_zero(void)const
	{
		return (x==0 && y==0 && z==0);
	}
	
	void print() const
	{
		Serial.print(x);
		Serial.print(F("\t"));
		Serial.print(y);
		Serial.print(F("\t"));
		Serial.println(z);
	}
	
	void print(uint8_t i) const
	{
		Serial.print(x, i);
		Serial.print(F("\t"));
		Serial.print(y, i);
		Serial.print(F("\t"));
		Serial.println(z, i);
	}
		
	vector3<T> operator + (const vector3<T> v)const;	
	vector3<T> operator - (const vector3<T> v)const;	
	vector3<T> operator - (void)const;	
	vector3<T> operator * (const T v) const;
	vector3<T> operator / (const T v) const;
	
	T operator * (const vector3<T> v) const; 		  // dot product
	vector3<T> operator % (const vector3<T> v) const; // cross product
	
	
	void normalise(void);	
	vector3<T>  unit(void)
	{
		T x_unit = x/magnitude();
		T y_unit = y/magnitude();
		T z_unit = z/magnitude();
		
		return vector3<T>(x_unit, y_unit, z_unit);
	}
};

typedef vector3<float>		vector3f;
typedef vector3<int16_t>	vector3i16;
typedef vector3<uint16_t>	vector3u16;
#endif