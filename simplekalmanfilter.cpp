#include "simplekalmanfilter.h"

using namespace sc;
using namespace matrix;
using namespace vector3_;

SimpleKalmanFilter::SimpleKalmanFilter()
{
	init();
}

void SimpleKalmanFilter::init()
{
	B = 0;
	Pk = 0;
	F = 1;
	uk = 0;
	Kk = 0;
	H = 1;
	Q = 1000;
	R = 1000;
	xk = 0;

	B_k.ident();
	P_k.clear();
	F_k.ident();
	u_k.clear();
	K_k.clear();
	H_k.ident();
	Q_k.diag(0.1);
	R_k.diag(0.1);
	x_k.clear();

	k = 0;
}

void SimpleKalmanFilter::correction(double zk)
{
	double xk1 = F * xk + B * uk;
	double Pk1 = F * Pk * F + Q;

	Kk = Pk * H / (H * Pk1 * H + R);
	xk = xk1 + Kk * (zk - H * xk1);
	Pk = (1 - Kk * H) * Pk1;
}

void SimpleKalmanFilter::set_zk(double zk)
{
	if(!k){
		xk = zk;
	}
	correction(zk);
	k++;
}

vector3_::Vector3d SimpleKalmanFilter::correction(const vector3_::Vector3d &zk)
{
	Matrix3d A, B;

	vector3_::Vector3d xk1 = F_k * x_k + B_k * u_k;
	A = F_k * P_k;
	A = A * F_k.t();
	matrix::Matrix3d P_k1 = A + Q_k;

	Vector3d yk = zk - H_k * xk1;
	A = H_k * P_k1;
	A = A * H_k.t();
	Matrix3d S_k = A + R_k;
	B = P_k1 * H_k.t();
	K_k = B * S_k.inv();
	x_k = xk1 + K_k * yk;
	P_k = (Matrix3d::I() - K_k * H_k) * P_k1;
	return x_k;
}

vector3_::Vector3d SimpleKalmanFilter::set_zk(const vector3_::Vector3d &zk)
{
	if(!k){
		x_k = zk;
	}
	k++;
	return correction(zk);
}
