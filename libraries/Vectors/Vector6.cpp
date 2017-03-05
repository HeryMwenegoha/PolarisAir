#include "Vector6.h"
template <typename T>
T vector6<T>::magnitude(void)
{
	return sqrt(x1*x1 + x2*x2 + x3*x3 + x4*x4 + x5*x5 + x6*x6); 
}

template <typename T>
vector6<T> vector6<T>::operator + (const vector6<T> v)const
{
	return vector6<T>(x1+v.x1, x2+v.x2, x3+v.x3, x4+v.x4, x5+v.x5, x6+v.x6);
}

template <typename T>
vector6<T> vector6<T>::operator - (const vector6<T> v)const
{
	return vector6<T>(x1-v.x1, x2-v.x2, x3-v.x3, x4-v.x4, x5-v.x5, x6-v.x6);
}

template <typename T>
T vector6<T>::operator * (const vector6<T> v)const
{
	return x1*v.x1 + x2*v.x2 + x3*v.x3 + x4*v.x4 + x5*v.x5 + x6*v.x6;
}

template <typename T>
vector6<T> vector6<T>::operator * (const T v)const
{
	return vector6<T>(v*x1, v*x2,v*x3,v*x4,v*x5,v*x6);
}	 // dot product


template float 			vector6<float>::magnitude(void);
template vector6<float> vector6<float>::operator + (const vector6<float> v)const;
template vector6<float> vector6<float>::operator - (const vector6<float> v)const;
template float 			vector6<float>::operator * (const vector6<float> v)const;
template vector6<float> vector6<float>::operator * (const float v)const;
