#ifndef MATRIX3
#define MATRIX3

#include <vector>

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

	Matrix3_(){
		std::fill(data, data + count, '\0');
	}
	Matrix3_(T *data){
		std::copy(data, data + count , this->data);
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
		return data[index * cols];
	}
	const T* operator [](int index) const{
		return &data[index * cols];
	}
	void set(int i0, int i1, T value){
		data[i0 * rows + i1] = value;
	}
	T& get(int i0, int i1){
		return data[i0 * cols + i1];
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

	}
	void ident(){
		std::fill(data, data + count, '\0');
		data[0] = data[4] = data[8] = 1;
	}
};

template< typename T >
sc::Vector3_< T > operator* (const Matrix3_< T > & m, sc::Vector3_< T > &v)
{
	sc::Vector3_< T > res;
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
				res[i][j] += m1[j][k] * m2[k][j];
			}
		}
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

typedef Matrix3_< float >	Matrix3f;
typedef Matrix3_< double >	Matrix3d;
typedef Matrix3_< int >		Matrix3i;

}

#endif // MATRIX3

