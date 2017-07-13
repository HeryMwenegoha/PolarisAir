#include "Arduino.h"
#include "Vector3.h"
template <typename T>
vector3<T>	vector3<T>::operator + (const vector3<T> v)const
{
	return (vector3<T>(x+v.x, y+v.y, z+v.z));
}

template <typename T>
vector3<T>	vector3<T>::operator - (const vector3<T> v)const
{
	return (vector3<T>(x-v.x, y-v.y, z-v.z));
}

template <typename T>
vector3<T>	vector3<T>::operator - (void)const
{
	return (vector3<T>(-x, -y, -z));
}

template <typename T>
vector3<T>	vector3<T>::operator * (const T v) const
{
	return (vector3<T>(x*v, y*v, z*v));
}

template <typename T>
vector3<T> vector3<T>::operator / (const T v) const
{
	return (vector3<T>(x/v, y/v, z/v));
}

template <typename T>
T	vector3<T>::operator * (const vector3<T> v) const
{
	return(x*v.x + y*v.y + z*v.z); // dot product
}

template <typename T>
vector3<T> vector3<T>::operator % (const vector3<T> v) const
{
	return vector3<T>((y*v.z - z*v.y), (z*v.x - x*v.z), (x*v.y - y*v.x)); // cross product
}

template <typename T>
void vector3<T>::normalise(void)
{
	T x_norm = x/magnitude();
	T y_norm = y/magnitude();
	T z_norm = z/magnitude();
	
	this->x = x_norm; this->y = y_norm;	this->z = z_norm;
}

template vector3<float>	vector3<float>::operator + (const vector3<float> v)const;
template vector3<float>	vector3<float>::operator - (const vector3<float> v)const;
template vector3<float>	vector3<float>::operator - (void)const;
template vector3<float>	vector3<float>::operator * (const float v) const;
template vector3<float>	vector3<float>::operator / (const float v) const;
template float			vector3<float>::operator * (const vector3<float> v) const;
template vector3<float>	vector3<float>::operator % (const vector3<float> v) const;
template void			vector3<float>::normalise(void);

template vector3<int8_t>	vector3<int8_t>::operator + (const vector3<int8_t> v)const;
template vector3<int8_t>	vector3<int8_t>::operator - (const vector3<int8_t> v)const;
template vector3<int8_t>	vector3<int8_t>::operator - (void)const;
template vector3<int8_t>	vector3<int8_t>::operator * (const int8_t v) const;
