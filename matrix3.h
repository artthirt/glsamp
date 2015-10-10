#ifndef MATRIX3
#define MATRIX3

#include <vector>
#include <iostream>
#include <sstream>

#include  "struct_controls.h"

namespace matrix{

template < typename T >
struct Matrix3_{
	enum{
		cols = 3,
		rows = 3,
		count = cols * rows,

	};
	T data[count];

	explicit Matrix3_(){
		std::fill(data, data + count, '\0');
	}
	explicit Matrix3_(T *data){
		std::copy(data, data + count , this->data);
	}
	explicit Matrix3_(T value){
		std::fill(data, data + count, value);
	}
	Matrix3_(const Matrix3_< T > &m){
		std::copy(m.data, m.data + count, data);
	}
	Matrix3_& operator= (const Matrix3_< T > &m){
		std::copy(m.data, m.data + count, data);
		return *this;
	}
	Matrix3_& operator*= (T val){
		for(int i = 0; i < count; i++)
			data[i] *= val;
		return *this;
	}
	T trace() const{
		T tr = 0;
		for(u_int i = 0; i < cols; i++)
			tr += data[i * cols];
		return tr;
	}
	T* operator [](int index) {
		return &data[index * cols];
	}
	const T* operator [](int index) const{
		return &data[index * cols];
	}
	inline void set(int i0, int i1, T value){
		data[i0 * rows + i1] = value;
	}
	inline T& get(int i0, int i1){
		return data[i0 * cols + i1];
	}
	inline const T& get(int i0, int i1) const{
		return data[i0 * cols + i1];
	}
	bool empty() const{
		T val = 0;
		for(int i = 0; i < count; i++) val += data[i] * data[i];
		return val < 1e-11;
	}
	Matrix3_< T > t() const{
		Matrix3_< T > res;
		for(int i = 0; i < cols; i++){
			for(int j = 0; j < rows; j++){
				res.data[j * rows + i] = data[i * cols + j];
			}
		}
		return res;
	}
	T det() const{
		double A = get(0, 0) * (get(1, 1) * get(2, 2) - get(1, 2) * get(2, 1));
		double B = get(0, 1) * (get(1, 0) * get(2, 2) - get(1, 2) * get(2, 0));
		double C = get(0, 2) * (get(1, 0) * get(2, 1) - get(1, 1) * get(2, 0));
		return A - B + C;
	}
	Matrix3_< T > inv(bool *ok = 0) const{
		Matrix3_ < T > res;

		bool _ok = true;
		bool& __ok = ok? *ok : _ok;

		if(empty()){
			__ok = false;
			return res;
		}

		T d = det();
		if(d < 1e-11){
			__ok = false;
			std::cout << "matrix not invertible\n";
			return res;
		}

		double A[] = {
			get(1, 1) * get(2, 2) - get(1, 2) * get(2, 1),
			get(0, 2) * get(2, 1) - get(0, 1) * get(2, 2),
			get(0, 1) * get(1, 2) - get(0, 2) * get(1, 1),

			get(1, 2) * get(2, 0) - get(1, 0) * get(2, 2),
			get(0, 0) * get(2, 2) - get(0, 2) * get(2, 0),
			get(0, 2) * get(1, 0) - get(0, 0) * get(1, 2),

			get(1, 0) * get(2, 1) - get(1, 1) * get(2, 0),
			get(0, 1) * get(2, 0) - get(0, 0) * get(2, 1),
			get(0, 0) * get(1, 1) - get(0, 1) * get(1, 0),
		};
		res = Matrix3_< T >(A);
		res *= 1.0 / d;
		return res;

	}
	T norm() const{
		T res = 0;
		for(int i = 0; i < cols; i++){
			for(int j = 0; j < rows; j++){
				res += data[i * cols + j] * data[i * cols + j];
			}
		}
		return sqrt(res);
	}
	void clear(){
		std::fill(data, data + count, '\0');
	}
	Matrix3_< T > ident(){
		clear();
		data[0] = data[4] = data[8] = 1;
		return *this;
	}
	void diag(T value){
		clear();
		data[0] = data[4] = data[8] = value;
	}
	void diag(T values[]){
		clear();
		for(int i = 0; i < rows; i++)
			data[i * cols + i] = values[i];
	}
	operator std::string() const{
		std::stringstream ss;
		ss << "[ ";
		for(int i = 0; i < rows; i++){
			ss << "  ";
			for(int j = 0; j < cols; j++){
				ss << get(i, j) << " ";
			}
			if(i != rows - 1)
				ss << "\n";
		}
		ss << " ]";
		return ss.str();
	}

	static Matrix3_< T > I(){
		Matrix3_< T > res;
		return res.ident();
	}
	static Matrix3_< T > rand(T multiply = 1){
		double d[count];
		for(int i = 0; i < count; i++){
			d[i] = ::rand();
		}
		Matrix3_< T > res(d);
		if(multiply != 1.){
			res *= multiply;
		}
		return res;
	}
};

template< typename T >
vector3_::Vector3_< T > operator* (const Matrix3_< T > & m, vector3_::Vector3_< T > &v)
{
	vector3_::Vector3_< T > res;
	for(int i = 0; i < Matrix3_<T>::rows; i++){
		for(int j = 0; j < Matrix3_<T>::cols; j++){
			res[i] += m[i][j] * v[j];
		}
	}
	return res;
}

template< typename T >
Matrix3_< T > operator* (const Matrix3_< T > & m1, const Matrix3_< T > &m2)
{
	Matrix3_< T > res;
	for(int i = 0; i < Matrix3_<T>::rows; i++){
		for(int j = 0; j < Matrix3_<T>::cols; j++){
			for(int k = 0; k < Matrix3_<T>::cols; k++){
				res[i][j] += m1[i][k] * m2[k][j];
			}
		}
	}
	return res;
}

template< typename T >
Matrix3_< T > operator* (const Matrix3_< T > & m1, double val)
{
	Matrix3_< T > res;
	for(int i = 0; i < Matrix3_<T>::count; i++){
		res.data[i] = m1.data[i] * val;
	}
	return res;
}

template< typename T >
Matrix3_< T > operator+ (const Matrix3_< T > & m1, const Matrix3_< T > &m2)
{
	Matrix3_< T > res;
	for(int i = 0; i < Matrix3_<T>::count; i++){
		res.data[i] = m1.data[i] + m2.data[i];
	}
	return res;
}

template< typename T >
Matrix3_< T > operator- (const Matrix3_< T > & m1, const Matrix3_< T > &m2)
{
	Matrix3_< T > res;
	for(int i = 0; i < Matrix3_<T>::count; i++){
		res.data[i] = m1.data[i] - m2.data[i];
	}
	return res;
}

template< typename T >
bool operator== (const Matrix3_< T > & m1, const Matrix3_< T > &m2)
{
	T val = 0;
	for(int i = 0; i < Matrix3_<T>::count; i++){
		T v1 = m1.data[i] - m2.data[i];
		val += v1 * v1;
	}
	return val < 1e-8;
}

template< typename T >
std::ostream& operator << (std::ostream& stream, const Matrix3_< T > & m)
{
	std::string s = m;
	stream << s;
	return stream;

}

typedef Matrix3_< float >	Matrix3f;
typedef Matrix3_< double >	Matrix3d;
typedef Matrix3_< int >		Matrix3i;

}

#endif // MATRIX3

