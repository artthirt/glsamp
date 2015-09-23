#ifndef SIMPLEKALMANFILTER_H
#define SIMPLEKALMANFILTER_H

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

	SimpleKalmanFilter(){
		init();
	}
	void init(){
		B = 0;
		Pk = 0;
		F = 1;
		uk = 0;
		Kk = 0;
		H = 1;
		Q = 200;
		R = 40;
		xk = 0;
		k = 0;
	}
	void correction(double zk){
		double xk1 = F * xk + B * uk;
		double Pk1 = F * Pk * F + Q;

		Kk = Pk * H / (H * Pk1 * H + R);
		xk = xk1 + Kk * (zk - H * xk1);
		Pk = (1 - Kk * H) * Pk1;
	}
	void set_zk(double zk){
		if(!k){
			xk = zk;
		}
		correction(zk);
		k++;
	}
};

#endif // SIMPLEKALMANFILTER_H
