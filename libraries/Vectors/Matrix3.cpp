#include "Matrix3.h"
template <typename T>
matrix3<T>	matrix3<T>::transpose(void)const
{
	vector3<T> a_T = vector3<T>(a.x, b.x, c.x);
	vector3<T> b_T = vector3<T>(a.y, b.y, c.y);
	vector3<T> c_T = vector3<T>(a.z, b.z, c.z);
	
	return (matrix3<T>(a_T, b_T, c_T));
}

template <typename T>
vector3<T> matrix3<T>::mul_transpose(const vector3<T> v)const
{
	matrix3<T> mat_transpose;
	vector3<T> result;
    mat_transpose = (*this).transpose();
	
	result = mat_transpose * v;
	return result;
}

template <typename T>
Vector2<T> matrix3<T>::mulXY(vector3<T> const v)const
{	
	return Vector2<T>(a*v, b*v);
}

template <typename T>
matrix3<T> matrix3<T>::operator *(const T v)const
{
	return (matrix3<T>(a*v, b*v, c*v));
}

template <typename T>
vector3<T> matrix3<T>::operator * (const vector3<T> v)const
{
	T _x_ef = (this->a) * v;
	T _y_ef = (this->b) * v;
	T _z_ef = (this->c) * v;
	
	return vector3<T>(_x_ef, _y_ef, _z_ef);
}

template <typename T>
matrix3<T> matrix3<T>::operator *(const matrix3<T> v)const
{
	vector3<T> a_T = vector3<T>(v.a.x, v.b.x, v.c.x);
	vector3<T> b_T = vector3<T>(v.a.y, v.b.y, v.c.y);
	vector3<T> c_T = vector3<T>(v.a.z, v.b.z, v.c.z);	
	
    matrix3<T> v_T = matrix3<T>(a_T, b_T, c_T);
	
	vector3<T> a_ = vector3<T>(a*v_T.a, a * v_T.b, a * v_T.c);
	vector3<T> b_ = vector3<T>(b*v_T.a, b * v_T.b, b * v_T.c);
	vector3<T> c_ = vector3<T>(c*v_T.a, c * v_T.b, c * v_T.c);
	
	return (matrix3<T>(a_, b_, c_));
}

template <typename T>
matrix3<T> matrix3<T>::operator +(const matrix3<T> v)const
{
	return (matrix3<T>(a+v.a, b+v.b, c+v.c));
}

template <typename T>
matrix3<T> matrix3<T>::operator -(const matrix3<T> v)const
{
	return (matrix3<T>(a-v.a, b-v.b, c-v.c));
}


template <typename T>
void matrix3<T>::from_euler(const T roll, const T pitch, const T yaw) 
{
	const T cr = cos(roll);
	const T cp = cos(pitch);
	const T cy = cos(yaw);
	const T sr = sin(roll);
	const T sp = sin(pitch);
	const T sy = sin(yaw);

	const T xe_xb = cp * cy;  const T xe_yb = sr * sp * cy - cr * sy;  const T xe_zb = cr * sp * cy + sr * sy;
	const T ye_xb = cp * sy;  const T ye_yb = sr * sp * sy + cr * cy;  const T ye_zb = cr * sp * sy - sr * cy;
	const T ze_xb = -sp    ;  const T ze_yb = sr * cp;                 const T ze_zb = cr * cp;  

	vector3<T> _vector_xe = vector3<T>(xe_xb, xe_yb, xe_zb);
	vector3<T> _vector_ye = vector3<T>(ye_xb, ye_yb, ye_zb);
	vector3<T> _vector_ze = vector3<T>(ze_xb, ze_yb, ze_zb); 

	a = _vector_xe;
	b = _vector_ye;
	c = _vector_ze;
}
 

template <typename T>
void matrix3<T>::rotate(const vector3<T> &v)
{
	matrix3<T>	TempMatrix;
	TempMatrix.a.x = a.y * v.z - a.z * v.y;
	TempMatrix.a.y = a.z * v.x - a.x * v.z;
	TempMatrix.a.z = a.x * v.y - a.y * v.x;
	
	TempMatrix.b.x = b.y * v.z - b.z * v.y;
	TempMatrix.b.y = b.z * v.x - b.x * v.z;
	TempMatrix.b.z = b.x * v.y - b.y * v.x;
	
	TempMatrix.c.x = c.y * v.z - c.z * v.y;
	TempMatrix.c.y = c.z * v.x - c.x * v.z;
	TempMatrix.c.z = c.x * v.y - c.y * v.x;
	
	(*this) = (*this) + TempMatrix;
}

template <typename T>
vector3<T> matrix3<T>::colx(void)const
{
	return vector3<T>(this->a.x, this->b.x, this->c.x);
}

template <typename T>
vector3<T> matrix3<T>::colz(void)const
{
	return vector3<T>(this->a.z, this->b.z, this->c.z);
}

template <typename T>
void matrix3<T>::identity(void)
{
	this->a = vector3<T>(1,0,0);
	this->b = vector3<T>(0,1,0);
	this->c = vector3<T>(0,0,1);
}


template matrix3<float>	matrix3<float>::transpose(void)const;
template vector3<float> matrix3<float>::mul_transpose(const vector3<float> v)const;
template Vector2<float> matrix3<float>::mulXY(vector3<float> const v)const;
template matrix3<float> matrix3<float>::operator *(const float v)const;
template vector3<float> matrix3<float>::operator *(const vector3<float> v)const;
template matrix3<float> matrix3<float>::operator *(const matrix3<float> v)const;
template matrix3<float> matrix3<float>::operator +(const matrix3<float> v)const;
template matrix3<float> matrix3<float>::operator -(const matrix3<float> v)const;
template void 			matrix3<float>::from_euler(const float roll, const float pitch, const float yaw); 
template void 			matrix3<float>::rotate(const vector3<float> &v);
template vector3<float> matrix3<float>::colx(void)const;
template vector3<float> matrix3<float>::colz(void)const;
template void 			matrix3<float>::identity(void);
