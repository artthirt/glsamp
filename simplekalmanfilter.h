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
	__int64 k;

	SimpleKalmanFilter(){
		init();
	}
	void init(){
		B = 0;
		Pk = 0;
		F = 10;
		uk = 0;
		Kk = 0;
		H = 10;
		Q = 20;
		R = 20;
		xk = 0;
		k = 0;
	}
	void prediction(){
		double xk1 = F * xk + B * uk;
		double Pk1 = F * Pk + Q;

		xk = xk1;
		Pk = Pk1;
	}
	void correction(double zk){
		Kk = Pk * H / (H * Pk * H + R);
		xk = xk + Kk * (zk - H * xk);
		Pk = (1 - Kk * H) * Pk;
	}
	void set_zk(double zk){
		if(!k){
			xk = zk;
		}
		prediction();
		correction(zk);
		k++;
	}
};

#endif // SIMPLEKALMANFILTER_H
