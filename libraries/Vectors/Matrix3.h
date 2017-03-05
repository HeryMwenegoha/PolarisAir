#ifndef Matrix3_h
#define Matrix3_h
#include "Arduino.h"
#include "Vector2.h"
#include "Vector3.h"
/*@ Library implemented by Hery A Mwenegoha (C) 20/02/2016
 */
template <typename T>
struct matrix3
{
	public:	
	vector3<T> a;
	vector3<T> b;
	vector3<T> c;
	
	matrix3<T>(){
		a = vector3<T>(0,0,0);
		b = vector3<T>(0,0,0);
		c = vector3<T>(0,0,0);
	}
	
	matrix3<T>(const vector3<T> ai, const vector3<T> bi, const vector3<T> ci){
		a = ai;
		b = bi;
		c = ci;
	}
	
	matrix3<T>(const T ax, const T ay, const T az, const T bx, const T by, const T bz, const T cx, const T cy, const T cz){
		a = vector3<T>(ax, ay, az);
		b = vector3<T>(bx, by, bz);
		c = vector3<T>(cx, cy, cz);
	}
	
	void rotate(const vector3<T> &v);
	
	bool is_nan(void)const
	{
			return (a.is_nan() || b.is_nan() || c.is_nan());
	}
			
	matrix3<T> transpose(void)const;
	
	vector3<T> mul_transpose(vector3<T> const v)const;
	
	Vector2<T> mulXY(vector3<T> const v)const;
	
	matrix3<T> operator * (const T v)const;
	
	vector3<T> operator * (const vector3<T> v)const;
	
	matrix3<T> operator * (const matrix3<T> v)const;
	
	matrix3<T> operator + (const matrix3<T> v)const;
	
	matrix3<T> operator - (const matrix3<T> v)const;
	
	void from_euler(const T roll, const T pitch, const T yaw);
	
	vector3<T> colx(void)const;
	
	vector3<T> colz(void)const;
	
	void identity(void);
		
};


typedef matrix3<float>	matrix3f;
typedef matrix3<int16_t>	matrix3i16;
typedef matrix3<uint16_t>	matrix3u16;
#endif