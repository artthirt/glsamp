#ifndef SIMPLEKALMANFILTER_H
#define SIMPLEKALMANFILTER_H

#include <struct_controls.h>
#include <matrix3.h>

class SimpleKalmanFilter
{
public:
	double F;
	double B;
	double Pk;
	double Q;
	double xk;
	double uk;
	double Kk;
	double H;
	double R;
	long long k;

	SimpleKalmanFilter();
	void init();
	void correction(double zk);
	void set_zk(double zk);

public:
	/// @link https://en.wikipedia.org/wiki/Kalman_filter
	matrix::Matrix3d F_k;
	matrix::Matrix3d B_k;
	matrix::Matrix3d P_k;
	matrix::Matrix3d K_k;
	matrix::Matrix3d H_k;
	matrix::Matrix3d Q_k;
	matrix::Matrix3d R_k;
	vector3_::Vector3d x_k;
	vector3_::Vector3d u_k;

	/// @brief may use. didn't set first x_k
	vector3_::Vector3d correction(const vector3_::Vector3d& zk);
	/// @brief use it
	vector3_::Vector3d set_zk(const vector3_::Vector3d& zk);

private:

};

#endif // SIMPLEKALMANFILTER_H
