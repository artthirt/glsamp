#include "mainwindow.h"
#include <QApplication>

#include "matrix3.h"

#include "time.h"

/// @test code
int test_matrix()
{
	using namespace matrix;

	time_t tm;
	time(&tm);
	srand(tm);

	double ar[Matrix3d::count] = {
		1, 2, 3,
		6, 4, 5,
		7, 3, 2
	}; /// det=9

	Matrix3d m(ar);

	double det = m.det();

	if(det != 9.){
		return 1;
	}

	std::string s = m;
	std::cout << s << "\n" << m.det() << "\n";
	s = (m * Matrix3d::I());
	std::cout << s << "\n";

	Matrix3d mi = m.inv();
	std::cout << mi << "\n";

	double d_inv[] = {
		-7./9., 5./9., -2./9.,
		23./9., -19./9., 13./9.,
		-10./9., 11./9., -8./9.
	};
	Matrix3d m_inv(d_inv);

	if(!(mi == m_inv)){
		return 2;
	}

	Matrix3d mm = m * mi;
	std::cout << mm << "\n";

	if(!(mm == Matrix3d::I())){
		return 3;
	}

	int verify = 0, zero_det = 0;
	for(int i = 0; i < 1000; i++){
		double d[9];
		for(int i = 0; i < 9; i++){
			d[i] = 100. * ((double)rand() / RAND_MAX + ((double)rand() / RAND_MAX - 0.5));
		}
		Matrix3d m(d), mi, mm;
		bool ok = true;
		mi = m.inv(&ok);
		if(!ok){
			zero_det++;
			continue;
		}
		mm = m * mi;
		if(mm == Matrix3d::I()){
			verify++;
		}
	}
	std::cout << "mat op inverse: verify=" << verify << "(ones); zero_det=" << zero_det << "\n";

	Matrix3d m1 = Matrix3d::rand(1.0 / RAND_MAX);
	Matrix3d m2 = Matrix3d::rand(1.0 / RAND_MAX);
	Matrix3d m3 = m1 + m2, m4 = m1 - m2, m5 = m1 - m1, m6 = m1 * 2., m7 = m1 + m1;

	std::cout << m1 << "\n" << m2 << "\n";

	if(m6 == m7){
		std::cout << "verify multiply & addition\n";
	}
	if(m5.empty()){
		std::cout << "verify subtraction\n";
	}
	std::cout << m3 << "\n" << m4;

	return 0;
}

int main(int argc, char *argv[])
{
	//test_matrix();

	QApplication a(argc, argv);
	MainWindow w;
	w.show();

	return a.exec();
}
