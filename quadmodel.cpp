#include "quadmodel.h"

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
#include <Windows.h>
#else
#include <chrono>
#endif

#include <QMatrix4x4>
#include <QDebug>
#include <QGLWidget>

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

void get_lever_axes(const QVector3D* v, QVector3D *vv)
{
	/// lever among 0 and 1	motor
	vv[0] = v[0] - v[1];
	/// lever among 2 and 3 motor
	vv[1] = v[2] - v[3];

//	/// vector from 0 to 1 motor
//	vv[0] = dn[0] + vv[0];
//	/// vector from 2 to 3 motor
//	vv[1] = dn[1] + vv[1];
}

//******************************************

QuadModel::QuadModel(QObject *parent):
	VirtGLObject(parent)
{
	m_lever = 1;
	m_mg = 9.8;
	m_max_power = 80;
	m_koeff_fade = 0.98;

	m_is_draw_telemetry = true;

	m_color = QColor(60, 255, 60, 255);

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
	std::tr1::random_device rd;
	generator = std::tr1::mt19937(rd);

	distribution = std::tr1::normal_distribution<double>(0.0, 0.00001);
	distribution_time = std::tr1::normal_distribution<double>(15, 15);
#else
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	generator = std::mt19937(seed);

	distribution = std::normal_distribution<double>(0.0, 0.0001);
	distribution_time = std::normal_distribution<double>(15, 15);
#endif

	m_delta_time = delta_time;
	m_time_noise.start();
	m_time_noise_start = m_time_noise.elapsed();

	connect(&m_timer_noise, SIGNAL(timeout()), this, SLOT(on_timeout_noise()));
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
	m_rot_speed = 0;

	m_delta_speed[0] = m_delta_speed[1] = QVector3D();

	m_speed = m_position = QVector3D();
	for(int i = 0; i < 4; i++) m_engines[i] = 0;
}

void QuadModel::reset_power()
{
	for(int i = 0; i < 4; i++) m_engines[i] = 0;
}

void QuadModel::set_distribution_parameters(double mean, double sigma)
{
#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
	distribution = std::tr1::normal_distribution<double>(mean, sigma);
#else
	distribution = std::normal_distribution<double>(mean, sigma);
#endif

}

void QuadModel::set_max_power(double value)
{
	m_max_power = value;
}

void QuadModel::set_draw_telemetry(bool value)
{
	m_is_draw_telemetry = value;
}

double QuadModel::engines_noise(int index)
{
	return qMax(0.0, m_engines[index] + m_engines_rnd[index]);
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

void QuadModel::set_koeff_fade(double value)
{
	m_koeff_fade = value;
}

double QuadModel::koeff_fade() const
{
	return m_koeff_fade;
}

void QuadModel::setControl(const StructControls &control)
{
	m_controls = control;
}

StructTelemetry QuadModel::telemetry() const
{
	return m_telemetry;
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

	draw_vect(normal_begin, Z0, Qt::yellow);
	draw_vect(course_begin, Z0, Qt::darkCyan);

	mat_type vals[] = {
		m_tmp_vc2.x(), m_tmp_vc2.y(), m_tmp_vc2.z(), 0,
		m_tmp_course.x(), m_tmp_course.y(), m_tmp_course.z(), 0,
		m_tmp_normal.x(), m_tmp_normal.y(), m_tmp_normal.z(), 0,
		0, 0, 0, 1
	};

	QMatrix4x4 m = QMatrix4x4(vals), mt;

	mt = m.transposed();

	QVector3D vt[4] = {
		QVector3D(0.2, 0.2, 0.2),
		QVector3D(-0.2, 0.2, 0.2),
		QVector3D(-0.2, -0.2, 0.2),
		QVector3D(0.2, -0.2, 0.2),
	};

	for(int i = 0; i < 4; i++){
		QVector3D vr = mt * vt[i];

		draw_vect(vr, Z0, Qt::white);
	}


	//qa.normalize();

	/*
	 *  angle = 2 * acos(qw)
	 *	x = qx / sqrt(1-qw*qw)
	 *	y = qy / sqrt(1-qw*qw)
	 *	z = qz / sqrt(1-qw*qw)
	 */

#ifdef QT4
	glMultMatrixd((mt.data()));
#elif defined(QT5)
	glMultMatrixf((mt.data()));
#endif

	draw_lever(QVector3D(0, wd_lv, 0), 45,	Qt::yellow);
	draw_lever(QVector3D(0, wd_lv, 0), 135,	Qt::green);
	draw_lever(QVector3D(0, wd_lv, 0), -45,	Qt::yellow);
	draw_lever(QVector3D(0, wd_lv, 0), -135,Qt::green);

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

	if(m_is_draw_telemetry)
		draw_telemetry();

	//draw_transp_plane(mt, QColor(30, 255, 30, 60));
	//draw_transp_plane(QMatrix4x4(), QColor(30, 30, 255, 60));
}

void QuadModel::tick()
{
	calc_trajectory();
}

QVector3D QuadModel::position() const
{
	return m_position;
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

/**
 * @brief rotate_lever
 * @param normal			- current normal
 * @param lever				- lever
 * @param diff_pow_engine	- ([0] + [1]) - ([2] + [3]) difference between power
 * @param course			- current course in -> new course out
 * @param vec				- current lever vectors in -> new lever vectors out
 */
void rotate_lever(const QVector3D& normal, double lever, double diff_pow_engine,
				   QVector3D &course, QVector3D* vec)
{
	QVector3D n[4], vn[4];
	for(int i = 0; i < 4; i++){
		n[i] = QVector3D::crossProduct(normal, vec[i]).normalized();
		n[i] *= diff_pow_engine;
		vn[i] = vec[i].normalized();
		vn[i] += n[i];
		vec[i] = lever * vn[i].normalized();
	}
	course = vec[0] + vec[2];
	course.normalize();
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

	double diff_pow_engine = engines_noise(0) + engines_noise(1);
	diff_pow_engine -= (engines_noise(2) + engines_noise(3));
	m_rot_speed += diff_pow_engine;
	m_rot_speed *= m_koeff_fade;
	rotate_lever(m_normal, m_lever, diff_pow_engine, m_course, v);

	double val = 0;
	for(int i = 0; i < 4; i++){
		n[i] = m_normal * engines_noise(i);
		val += engines_noise(i);
	}

	QVector3D an = m_normal * val;
	m_speed += 0.1 * an;
	m_speed -= QVector3D(0, 0, 0.01);
	m_speed *= m_koeff_fade;

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

	draw_vect(m_tmp_normal, Z0, QC(0.7f, 1.f, 0.3f));
	draw_vect(m_tmp_vc2, Z0, QC(1.f, 0.3f, 0.3f));
	draw_vect(m_tmp_course, Z0, QC(0.3f, 0.3f, 1.f));

	glLineWidth(3);

//	glColor3f(1, 0.3, 1);
//	for(int i = 0; i < 2; i++){
//		draw_vect(m_tmp_vv[i], Z0, QC(1, 0.3, 1));
//	}

	QVector3D v[4];
	get_vec_levers(m_tmp_course, m_tmp_normal, m_lever, v);

	glColor3f(0.f, 0.3f, 1.f);
	for(int i = 0; i < 4; i++){
		draw_vect(m_tmp_n[i], v[i], QC(0.f, 0.3f, 1.f));
	}


	for(int i = 0; i < 4; i++){
		draw_vect(v[i], Z0, QC(0.f, 1.f, 0.3f));
	}
	glLineWidth(1);

//	m_normal = normal_begin;
//	m_course = course_begin;

}

double detv3(const QVector3D& v1, const QVector3D& v2, const QVector3D& v3)
{
	const mat_type vals[] = {
		v1.x(), v1.y(), v1.z(), 0,
		v2.x(), v2.y(), v2.z(), 0,
		v3.x(), v3.y(), v3.z(), 0,
		0,		0,		0,		1
	};
	return QMatrix4x4(vals).determinant();
}

void QuadModel::draw_telemetry()
{
	glPushMatrix();

	glLoadIdentity();

	glTranslatef(0.4, 0.35, -1.01f);

	const int cnt_circle = 50;
	const float R = 0.1;

	glLineWidth(2);
	glColor3f(1, 1, 1);

	glBegin(GL_TRIANGLE_STRIP);
	for(int i = 0; i <= cnt_circle; i++){
		float x, y;

		x = R * sin(2.0 * M_PI * i/cnt_circle);
		y = R * cos(2.0 * M_PI * i/cnt_circle);

		glVertex3f(0, 0, -0.001);
		glVertex3f(x, y, -0.001);
	}
	glEnd();

	glColor3f(1, 1, 1);
	glLineWidth(2);
	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < cnt_circle; i++){
		float x, y;

		x = R * sin(2.0 * M_PI * i/cnt_circle);
		y = R * cos(2.0 * M_PI * i/cnt_circle);

		glVertex3f(x, y, 0);
	}
	glEnd();

	QVector3D tmp_vc2_z0 = m_tmp_vc2;
	tmp_vc2_z0.setZ(0);

	QVector3D nc = m_tmp_course;
	nc.setZ(0);

	QVector3D v1 = QVector3D::crossProduct(normal_begin, m_tmp_course);

	double d = QVector3D::dotProduct(m_tmp_vc2, v1);
	double d1 = 0;

//	if(m_tmp_normal.z() < 0){
//		b = M_PI + b;
//	}

	QGLWidget *w = dynamic_cast< QGLWidget* >(parent());
	if(w){
		glColor3f(1, 1, 1);
		w->renderText(-1.4, 0.6, -1.0, QString::number(d, 'f', 3) + " " + QString::number(d1, 'f', 3), QFont("Arial", 14));
	}

	float l = -tmp_vc2_z0.length();
	float lc = -nc.length();
	if(d < 0){
		l = -l;
		lc = -lc;
	}

	QVector3D vl = QVector3D(l, m_tmp_vc2.z(), 0).normalized() * R;

	QVector3D vp = QVector3D(vl.y(), -vl.x(), 0).normalized();

	double offset_course = 0;
	{
		QVector3D vc = QVector3D(lc, m_tmp_course.z(), 0).normalized() * R;
		vc.setX(0);
		double l = vc.y();
		offset_course = l;
		glColor3f(0.7, 0.7, 0.7);
		glLineWidth(4);
		QVector3D v0 = vp * l, v1, v2;
		l = v0.length();
		if(l < R){
			l = sqrt(R * R - l * l)/R;
			v1 = v0 - l * vl, v2 = v0 + l * vl;

			glBegin(GL_LINES);
			glVertex3f(v1.x(), v1.y(), v1.z());
			glVertex3f(v2.x(), v2.y(), v2.z());
			glEnd();
		}

		glColor3f(1, 0, 0);
		/// down tangaj arc
		glBegin(GL_TRIANGLE_STRIP);
		QVector3D vl0 = vl.normalized();
		for(int i = 0; i <= 60; i++){
			QVector3D vz = vp * (offset_course - i/60.0 * R * 2), v3, v4;

			l = vz.length();
			if(l <= R){
				l = sqrt(R * R - l * l);
				v3 = vz - l * vl0, v4 = vz + l * vl0;
				glVertex3f(v3.x(), v3.y(), -0.0005);
				glVertex3f(v4.x(), v4.y(), -0.0005);
			}else{
				glVertex3f(vz.x(), vz.y(), -0.0005);
			}
		}
		glEnd();
	}
//	glColor3f(0, 0.3, 1);
//	glBegin(GL_LINES);
//	glVertex3f(0, 0, 0);
//	glVertex3f(vc.x(),vc.y(), vc.z());
//	glEnd();


	glLineWidth(2);
	vl.normalize();
	for(int i = 0; i < 7; i++){
		QVector3D v0 = vp * (offset_course + i/3.0 * R), v1, v2, v3, v4;
		double l = v0.length();
		if(l < R){
			l = sqrt(R * R - l * l)/3;
			v1 = v0 - l * vl, v2 = v0 + l * vl;
			/// scale of up tangaj
			glColor3f(1, 0, 0);
			glBegin(GL_LINES);
			glVertex3f(v1.x(), v1.y(), v1.z());
			glVertex3f(v2.x(), v2.y(), v2.z());
			glEnd();
		}

		QVector3D vz = vp * (offset_course - i/3.0 * R);

		l = vz.length();
		if(l < R){
			l = sqrt(R * R - l * l)/3;
			v3 = vz - l * vl, v4 = vz + l * vl;
			/// scale of down tangaj
			glColor3f(1, 0.7, 0);
			glBegin(GL_LINES);
			glVertex3f(v3.x(), v3.y(), v3.z());
			glVertex3f(v4.x(), v4.y(), v4.z());
			glEnd();
		}
	}

	glLineWidth(1);
	glPopMatrix();

	draw_vect(tmp_vc2_z0, Z0);
	draw_vect(QVector3D(0, 0, m_tmp_vc2.z()), tmp_vc2_z0);
}

void QuadModel::draw_transp_plane(const QMatrix4x4 &matrix, const QColor &c)
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//	glBlendEquation(GL_FUNC_ADD);

	glPushMatrix();

#ifdef QT4
	glMultMatrixd((matrix.data()));
#elif defined(QT5)
	glMultMatrixf((matrix.data()));
#endif

	glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF());

	glBegin(GL_QUADS);
	glVertex3f(1, 1, 0);
	glVertex3f(-1, 1, 0);
	glVertex3f(-1, -1, 0);
	glVertex3f(1, -1, 0);
	glEnd();

	glPopMatrix();

	glDisable(GL_BLEND);
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
