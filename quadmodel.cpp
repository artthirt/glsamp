#include "quadmodel.h"

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
#include <Windows.h>
#else
#include <chrono>
#endif

#include <QDebug>

#include <GL/gl.h>

#include "math.h"

const double wd_lv = 0.05;

const int delta_time = 200;

const QVector3D normal_begin(0, 0, 1);
const QVector3D course_begin(0, 1, 0);

const int max_trj_pts = 10000;

const QVector3D Z0 = QVector3D();

//*****************************


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

//******************************************

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

	distribution = std::tr1::normal_distribution<double>(0.0005, 0.0005);
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
	m_normal = normal_begin;
	m_course = course_begin;

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

	glTranslated(m_position.x(), m_position.y(), m_position.z());

	glColor3d(m_color.redF(), m_color.greenF(), m_color.blueF());

	QQuaternion qa, qb;

	double a = QVector3D::dotProduct(course_begin, m_course);
	bool fa = false;
	if(qFuzzyCompare(qAbs(a), 1)){
		fa = true;
	}else{
		QVector3D rotV = QVector3D::crossProduct(course_begin, m_course).normalized();
		a = acos(a) * 180.0 / M_PI;
		qa = QQuaternion::fromAxisAndAngle(rotV, a);
		//QQuaternion q = GQ(a, rotV);
		//glRotated(a, rotV.x(), rotV.y(), rotV.z());
	}

	QVector3D v0 = QVector3D::crossProduct(normal_begin, course_begin),
			v1 = QVector3D::crossProduct(m_normal, m_course);

	double b = QVector3D::dotProduct(v0, v1);
	bool fb = false;
	if(qFuzzyCompare(qAbs(b), 1)){
		fb == true;
	}else{
		QVector3D rotV = QVector3D::crossProduct(v0, v1).normalized();
		b = acos(b) * 180.0 / M_PI;
		qb = QQuaternion::fromAxisAndAngle(rotV, b);
//		glRotated(b, rotV.x(), rotV.y(), rotV.z());
	}

	if(fa){
		if(fb){

		}else{
			qa = qb;
		}
	}else{
		if(fb){

		}else{
			qa *= qb;
		}
	}
	qa = qa * qb;
	//qa.normalize();

	/*
	 *  angle = 2 * acos(qw)
	 *	x = qx / sqrt(1-qw*qw)
	 *	y = qy / sqrt(1-qw*qw)
	 *	z = qz / sqrt(1-qw*qw)
	 */

	a = 2 * acos(qa.scalar()) * 180.0 / M_PI;
	double dd;
	QVector3D xd;
	dd = sqrt( 1 - qa.scalar() * qa.scalar());
	if(!qFuzzyIsNull(dd)){
		xd = QVector3D( qa.x() / dd,
						qa.y() / dd,
						qa.z() / dd);
		xd.normalize();

		glRotated(a, xd.x(), xd.y(), xd.z());
	}

//	draw_lever(QVector3D(0, wd_lv, 0), 45,	Qt::yellow);
//	draw_lever(QVector3D(0, wd_lv, 0), 135,	Qt::green);
//	draw_lever(QVector3D(0, wd_lv, 0), -45,	Qt::yellow);
//	draw_lever(QVector3D(0, wd_lv, 0), -135,Qt::green);

	glLineWidth(1);

	glPopMatrix();

	glPointSize(4);
	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	foreach (QVector3D pt, m_trajectory) {
		glVertex3d(pt.x(), pt.y(), pt.z());
	}
	glEnd();

	draw_tmp_struct();
}

void QuadModel::tick()
{
	calc_trajectory();
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

void get_vec_levers(const QVector3D &course, const QVector3D &normal, double lever, QVector3D* vout)
{
	QVector3D tang;

	/// tangaj quad
	tang = QVector3D::crossProduct(normal, course);
	/// get vectors of levers
	vout[0] = course + tang;
	vout[2] = course - tang;
	vout[0].normalize();
	vout[0] *= lever;

	vout[2].normalize();
	vout[2] *= lever;

	vout[1] = -vout[0];
	vout[3] = -vout[2];
}

void QuadModel::calc_trajectory()
{
	/*	scheme of indexes motors
	 * 		0		2
	 *		  \   /
	 *			-
	 *		  /   \
	 *		3		1
	*/

	QVector3D v[4], n[4], dn[2], vv[2];


	get_vec_levers(m_course, m_normal, m_lever, v);

	double val = 0;
	for(int i = 0; i < 4; i++){
		n[i] = m_normal * engines/*_noise*/(i);
		val += engines/*_noise*/(i);
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

	/// lever among 0 and 1	motor
	vv[0] = v[0] - v[1];
	/// lever among 2 and 3 motor
	vv[1] = v[2] - v[3];

	/// vector from 0 to 1 motor
	vv[0] = dn[0] + vv[0];
	/// vector from 2 to 3 motor
	vv[1] = dn[1] + vv[1];

	for(int i = 0; i < 2; i++){
		m_tmp_vv[i] = vv[i];
	}

	QVector3D vnorm = QVector3D::crossProduct(vv[1], vv[0]).normalized();
	QVector3D vcourse = vv[0] + vv[1];
	QVector3D vc2 = QVector3D::crossProduct(vnorm, vcourse).normalized();

	vcourse.normalize();

	m_tmp_normal = vnorm;
	m_tmp_course = vcourse;
	m_tmp_vc2 = vc2;

	m_normal = vnorm;
	m_course = vcourse;

	for(int i = 0; i < 4; i++){
		m_tmp_n[i] = n[i];
	}

	qDebug() << "---";

}

void QuadModel::draw_tmp_struct()
{
	glLineWidth(5);

	draw_vect(m_tmp_normal, Z0, QC(0.3, 1, 0.3));
	draw_vect(m_tmp_vc2, Z0, QC(1, 0.3, 0.3));
	draw_vect(m_tmp_course, Z0, QC(0.3, 0.3, 1));

	glLineWidth(3);

//	glColor3f(1, 0.3, 1);
//	for(int i = 0; i < 2; i++){
//		draw_vect(m_tmp_vv[i], Z0, QC(1, 0.3, 1));
//	}

	QVector3D v[4];
	get_vec_levers(m_tmp_course, m_tmp_normal, m_lever, v);

	glColor3f(0, 0.3, 1);
	for(int i = 0; i < 4; i++){
		draw_vect(m_tmp_n[i], v[i], QC(0, 0.3, 1));
	}


	for(int i = 0; i < 4; i++){
		draw_vect(v[i], Z0, QC(0, 1, 0.3));
	}
	glLineWidth(1);

	m_normal = normal_begin;
	m_course = course_begin;

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

void QuadModel::draw_lever(QVector3D offset, double angleXY, const QColor col)
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

	glColor3d( col.redF(), col.greenF(), col.blueF() );
	draw_plane(plane1);
	draw_plane(plane2);
	draw_plane(plane3);
	draw_plane(plane4);

	glPopMatrix();
}
