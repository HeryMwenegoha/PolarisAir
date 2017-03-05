#include "Vector2.h"
template <typename T>
float Vector2<T>::length()
{
    return sqrt(x * x + y * y);
}

template <typename T>
void Vector2<T>::normalise()
{
    float xn = x/length();
    float yn = y/length();
	
	this->x = xn;
	this->y = yn;
}

template <typename T>
Vector2<T> Vector2<T>::normalized(){
	float xn = x/length();
    float yn = y/length();
	return(Vector2<T>(xn, yn));
}

template <typename T>
Vector2<T> Vector2<T>::operator * (const T v) const
{ 
    return(Vector2<T>(x * v, y * v));
}
  
template <typename T>
T Vector2<T>::operator * (const Vector2<T> v)const
{
    return x*v.x + y*v.y;
}
  
template <typename T>
T Vector2<T>::operator % (const Vector2<T> v)const
{
    return x*v.y - y*v.x;
}

template <typename T>
Vector2<T> Vector2<T>::operator - (void) const
{
    T xn = -x;
    T yn = -y;
    return Vector2<T>(xn, yn);
}  


template float          Vector2<float>::length();
template void		    Vector2<float>::normalise();
template Vector2<float> Vector2<float>::normalized();
template Vector2<float> Vector2<float>::operator * (const float v) const;
template float          Vector2<float>::operator * (const Vector2<float> v)const;
template float          Vector2<float>::operator % (const Vector2<float> v)const;
template Vector2<float> Vector2<float>::operator - (void) const;