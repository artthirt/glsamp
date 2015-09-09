#include "quadmodel.h"

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
#include <Windows.h>
#else
#include <chrono>
#endif

#include <QDebug>

#include <GL/gl.h>

#include "math.h"

const double wd_lv = 0.1;

const int delta_time = 200;

QuadModel::QuadModel()
{
	m_lever = 1;
	m_mg = 9.8;
	m_max_power = 80;
	m_koeff = 1;

	m_color = QColor(60, 255, 60, 255);

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
	std::tr1::random_device rd;
	generator = std::tr1::mt19937(rd);

	distribution = std::tr1::normal_distribution<double>(0.0001, 0.0001);
	distribution_time = std::tr1::normal_distribution<double>(15, 15);
#else
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	generator = std::mt19937(seed);

	distribution = std::normal_distribution<double>(0.005, 0.005);
	distribution_time = std::normal_distribution<double>(15, 15);
#endif

	m_delta_time = delta_time;
	m_time_noise.start();
	m_time_noise_start = m_time_noise.elapsed();

	bool res = connect(&m_timer_noise, SIGNAL(timeout()), this, SLOT(on_timeout_noise()));
	m_timer_noise.start(10);

	generate_engines_rnd();

	reset();
}

double QuadModel::lever() const
{
	return m_lever;
}

void QuadModel::setLever(double value)
{
	m_lever = value;
}

void QuadModel::setAlpha(double value)
{
	m_alpha = value;
}

void QuadModel::setBetha(double value)
{
	m_betha = value;
}

void QuadModel::setGamma(double value)
{
	m_gamma = value;
}

double QuadModel::power() const
{
	double pw = 0;
	for(int i = 0; i < 4; i++){
		pw += m_engines[i] + m_engines_rnd[i];
	}
	return pw;
}

double QuadModel::real_power() const
{
	double pw = 0;
	for(int i = 0; i < 4; i++){
		pw += m_engines[i];
	}
	return pw;
}


void QuadModel::reset()
{
	m_normal = QVector3D(0, 0, 1);
	m_course = QVector3D(0, 1, 0);

	m_delta_speed[0] = m_delta_speed[1] = QVector3D();

	m_acceleration = 0;
	m_acceleration_mg = 0;
	m_alpha = m_betha = m_gamma = 0;
	m_speed = m_position = QVector3D();
	for(int i = 0; i < 4; i++) m_engines[i] = 0;
}

void QuadModel::reset_power()
{
	for(int i = 0; i < 4; i++) m_engines[i] = 0;
}

double QuadModel::acceleration() const
{
	return m_acceleration;
}

double QuadModel::acceleration_mg() const
{
	return m_acceleration_mg;
}

void QuadModel::set_max_power(double value)
{
	m_max_power = value;
}

double QuadModel::mg() const
{
	return m_mg;
}

void QuadModel::set_mg(double mg)
{
	m_mg = mg;
}

void QuadModel::set_koeff(double value)
{
	m_koeff = value;
}

double QuadModel::koeff() const
{
	return m_koeff;
}

void QuadModel::add_alpha(double value)
{
	m_alpha += value;
}

void QuadModel::add_betha(double value)
{
	m_betha += value;
}

void QuadModel::add_gamma(double value)
{
	m_gamma += value;
}

void QuadModel::add_power(double value)
{
	for(int i = 0; i < 4; i++){
		m_engines[i] += value;
		if(m_engines[i] > m_max_power){
			m_engines[i] = m_max_power;
		}
		if(m_engines[i] < 0)
			m_engines[i] = 0;
	}
}

void QuadModel::add_power(int index, double value)
{
	m_engines[index] += value;
	if(m_engines[index] < 0)
		m_engines[index] = 0;
	if(m_engines[index] > m_max_power)
		m_engines[index] = m_max_power;
}

void QuadModel::set_power(double value)
{
	for(int i = 0; i < 4; i++){
		m_engines[i] = value + m_engines_rnd[i];
		if(m_engines[i] > m_max_power){
			m_engines[i] = m_max_power;
		}
		if(m_engines[i] < 0)
			m_engines[i] = 0;
	}
}

void QuadModel::setColor(const QColor &color)
{
	m_color = color;
}

QColor QuadModel::color() const
{
	return m_color;
}


void QuadModel::init()
{
	reset();
}

void QuadModel::draw()
{
	glPushMatrix();
	glLineWidth(3);

//	glTranslated(m_position.x(), m_position.y(), m_position.z());

//	glColor3d(m_color.redF(), m_color.greenF(), m_color.blueF());

//	draw_leter(QVector3D(0, wd_lv, 0), 45);
//	draw_leter(QVector3D(0, wd_lv, 0), 135);
//	draw_leter(QVector3D(0, wd_lv, 0), -45);
//	draw_leter(QVector3D(0, wd_lv, 0), -135);
	calc_trajectory();

	glLineWidth(1);
	glPopMatrix();
}

const QVector3D Z0 = QVector3D();

static inline QColor QC(float r, float g, float b)
{
	return QColor(r * 255, g * 255, b * 255);
}

void draw_vect(const QVector3D& v, const QVector3D& offs = QVector3D(), const QColor c = QColor(255, 60, 0, 255))
{
	QVector3D offsv = offs + v;

	glColor3f(c.redF(), c.greenF(), c.blueF());
	glBegin(GL_LINES);
	glVertex3d(offs.x(), offs.y(), offs.z());
	glVertex3d(offsv.x(), offsv.y(), offsv.z());
	glEnd();
}

QQuaternion GQ(double angle, const QVector3D& v)
{
	if(qFuzzyIsNull(angle) || qFuzzyIsNull(v.length())){
		return QQuaternion(1, 0, 0, 0);
	}
	double ac = cos(angle/2), as = sin(angle/2);
	double x, y, z;
	x = v.x() / as;
	y = v.y() / as;
	z = v.z() / as;
	return QQuaternion(ac, x, y, z);
}

const int max_trj_pts = 10000;

void QuadModel::tick()
{

}

void QuadModel::on_timeout_noise()
{
	double delta = m_time_noise.elapsed() - m_time_noise_start;
	if(delta > m_delta_time){
		delta = 0;
		m_time_noise.restart();
		m_time_noise_start = m_time_noise.elapsed();
		m_delta_time = delta_time + distribution_time(generator);

		change_engines_rnd();
	}

	delta /= m_delta_time;

	calc_engines_rnd(delta);
}

void QuadModel::change_engines_rnd()
{
	for(int i = 0; i < 4; i++){
		m_cur_engines_rnd[i] = m_next_engines_rnd[i];
		m_next_engines_rnd[i] = distribution(generator);
	}
}

void QuadModel::calc_trajectory()
{
	/*	scheme of indexes motors
	 * 		2		0
	 *		  \   /
	 *			-
	 *		  /   \
	 *		1		3
	*/

	QVector3D tang, v[4], v5, n[4], dn[2], vv[2];

	/// tangaj quad
	tang = QVector3D::crossProduct(m_normal, m_course);
	/// get vectors of leters
	v[0] = m_course + tang;
	v[2] = m_course - tang;
	v[0].normalize();
	v[0] *= m_lever;

	v[2].normalize();
	v[2] *= m_lever;

	v[1] = -v[0];
	v[3] = -v[2];

	double val = 0;
	for(int i = 0; i < 4; i++){
		n[i] = m_normal * engines_noise(i);
		val += engines_noise(i);
	}

	QVector3D an = m_normal * val;
	m_speed += 0.1 * an;
	m_speed -= QVector3D(0, 0, 0.01);
	m_speed *= 0.98;

	QVector3D tp = m_position + m_speed;
	if(tp.z() < 0){
		tp.setZ(0);
		tp.setX(m_position.x());
		tp.setY(m_position.y());
	}else{
		if(qFuzzyIsNull(tp.z())){
			tp.setX(m_position.x());
			tp.setY(m_position.y());
		}
	}
	m_position = tp;

	if(m_trajectory.size()){
		QVector3D lp = m_trajectory.last();
		if(lp != m_position){
			m_trajectory.push_back(m_position);
		}
	}else{
		m_trajectory.push_back(m_position);
	}

	if(m_trajectory.size() > max_trj_pts){
		m_trajectory.pop_front();
	}

	/// delta power from 0 to 1 motor
	dn[0] = n[0] - n[1];
	/// delta power from 2 to 3 motor
	dn[1] = n[2] - n[3];

	m_delta_speed[0] += dn[0];
	m_delta_speed[1] += dn[1];

	dn[0] = m_delta_speed[0];
	dn[1] = m_delta_speed[1];

	for(int i = 0; i < 2; i++)
		m_delta_speed[i] *= 0.95;

	/// leter among 0 and 1	motor
	vv[0] = v[0] - v[1];
	/// leter among 2 and 3 motor
	vv[1] = v[2] - v[3];

	/// vector from 0 to 1 motor
	vv[0] = dn[0] + vv[0];
	/// vector from 2 to 3 motor
	vv[1] = dn[1] + vv[1];

	glColor3f(1, 0.3, 1);
	for(int i = 0; i < 2; i++){
		draw_vect(vv[i], Z0, QC(1, 0.3, 1));
	}

	QVector3D vc1 = QVector3D::crossProduct(vv[1], vv[0]).normalized();
	QVector3D va1 = vv[0] + vv[1];
	QVector3D vc2 = QVector3D::crossProduct(vc1, va1).normalized();

	va1.normalize();

	draw_vect(vc1, Z0, QC(0.3, 1, 0.7));
	draw_vect(vc2, Z0, QC(0.3, 1, 0.7));
	draw_vect(va1, Z0, QC(0.3, 1, 0.7));

	m_normal = vc1;
	m_course = va1;

	qDebug() << "---";

	glPushMatrix();

	glTranslated(m_position.x(), m_position.y(), m_position.z());

	glColor3f(0, 0.3, 1);
	for(int i = 0; i < 4; i++){
		draw_vect(n[i], v[i], QC(0, 0.3, 1));
	}

	for(int i = 0; i < 4; i++){
		draw_vect(v[i], Z0, QC(0, 1, 0.3));
	}
	glPopMatrix();

	glPointSize(4);
	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	foreach (QVector3D pt, m_trajectory) {
		glVertex3d(pt.x(), pt.y(), pt.z());
	}
	glEnd();
}

void QuadModel::generate_engines_rnd()
{
	for(int i = 0; i < 4; i++){
		m_cur_engines_rnd[i] = distribution(generator);
		m_next_engines_rnd[i] = distribution(generator);
		m_engines_rnd[i] = m_cur_engines_rnd[i];
	}
}

void QuadModel::calc_engines_rnd(double delta)
{
	double rp = real_power();
	bool zero_rp = qFuzzyIsNull(rp);

	for(int i = 0; i < 4; i++){
		m_engines_rnd[i] = zero_rp? 0 : (m_cur_engines_rnd[i] + delta * (m_next_engines_rnd[i] - m_cur_engines_rnd[i]));
	}
}

typedef double Vector3D[3];

void draw_plane(const Vector3D values[4])
{
//	const int indexes[] = {
//		0, 1, 3,
//		1, 2, 3
//	};

	struct Triangles{
		int A, B, C;
		Triangles(){
			A = B = C = 0;
		}
		Triangles(int A, int B, int C){
			this->A = A;
			this->B = B;
			this->C = C;
		}
	};

	const Triangles indexes[] = {
		Triangles(0, 1, 3),
		Triangles(1, 2, 3),
	};

//	std::vector< Triangles > indexes;

//	for(int i = 0; i < w_count - 1; i++){
//		for(int j = 0; j < h_count - 1; j++){
//			indexes.push_back(Triangles(
//				(i)		* w_count + (j),
//				(i + 1) * w_count + (j),
//				(i + 1)	* w_count + (j + 1)
//			));
//			indexes.push_back(Triangles(
//				(i + 1)	* w_count + (j),
//				(i) * w_count + (j + 1),
//				(i)	* w_count + (j)
//			));
//		}
//	}

	glBegin(GL_TRIANGLES);
	for(int i = 0; i < 2; i++){
		const Vector3D &v1 = values[indexes[i].A];
		const Vector3D &v2 = values[indexes[i].B];
		const Vector3D &v3 = values[indexes[i].C];
		glVertex3dv((GLdouble*)&v1);
		glVertex3dv((GLdouble*)&v2);
		glVertex3dv((GLdouble*)&v3);
	}
	glEnd();
}

void QuadModel::draw_leter(QVector3D offset, double angleXY)
{
	const Vector3D plane1[] = {
		{-wd_lv,	0,			wd_lv},
		{-wd_lv,	m_lever,	wd_lv},
		{wd_lv,		m_lever,	wd_lv},
		{wd_lv,		0,			wd_lv}
	};
	const Vector3D plane2[] = {
		{-wd_lv,	0,			-wd_lv},
		{-wd_lv,	m_lever,	-wd_lv},
		{wd_lv,		m_lever,	-wd_lv},
		{wd_lv,		0,			-wd_lv}
	};
	const Vector3D plane3[] = {
		{wd_lv,		0,			-wd_lv},
		{wd_lv,		m_lever,	-wd_lv},
		{wd_lv,		m_lever,	wd_lv},
		{wd_lv,		0,			wd_lv}
	};
	const Vector3D plane4[] = {
		{-wd_lv,	0,			-wd_lv},
		{-wd_lv,	m_lever,	-wd_lv},
		{-wd_lv,	m_lever,	wd_lv},
		{-wd_lv,	0,			wd_lv}
	};

	glPushMatrix();

	glRotated(angleXY, 0, 0, 1);
	glTranslated(offset.x(), offset.y(), offset.z());

	draw_plane(plane1);
	draw_plane(plane2);
	draw_plane(plane3);
	draw_plane(plane4);

	glPopMatrix();
}
