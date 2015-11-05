#include "gyrodata.h"

#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QByteArray>
#include <QStringList>
#include <QColor>
#include <QDebug>
#include <QApplication>
#include <QGLWidget>

#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>

#include <QUdpSocket>
#include <QDataStream>
#include <QThreadPool>

#include "writelog.h"

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
#include <Windows.h>
#else
#include <chrono>
#endif

#include <GL/gl.h>

#include <global.h>
#include <simple_xml.hpp>

#include "calibrateaccelerometer.h"

using namespace sc;
using namespace vector3_;
using namespace quaternions;

///////////////////////////////

const int regTelem	= qRegisterMetaType< sc::StructTelemetry >("sc::StructTelemetry");
const int regVec3d	= qRegisterMetaType< vector3_::Vector3d >("vector3_::Vector3d");
const int regVec3i	= qRegisterMetaType< vector3_::Vector3i >("vector3_::Vector3i");

const QString xml_config("gyro.xml");

///////////////////////////////

QMatrix4x4 fromQuaternion(const Quaternion& q_in)
{
	Quaternion q = q_in.normalized();
	mat_type qi2 = q.x() * q.x();
	mat_type qj2 = q.y() * q.y();
	mat_type qk2 = q.z() * q.z();

	mat_type qij = q.x() * q.y();
	mat_type qik = q.x() * q.z();
	mat_type qjk = q.y() * q.z();

	mat_type qir = q.w * q.x();
	mat_type qjr = q.w * q.y();
	mat_type qkr = q.w * q.z();

	mat_type data[] = {
		1.0 - 2.0 * (qj2 + qk2), 2.0 * (qij - qkr), 2.0 * (qik + qjr), 0.0,
		2.0 * (qij + qkr), 1.0 - 2.0 * (qi2 + qk2), 2.0 * (qjk - qir), 0.0,
		2.0 * (qik - qjr), 2.0 * (qjk + qir), 1.0 - 2.0 * (qi2 + qj2), 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	QMatrix4x4 m(data, 4, 4);

	return m.transposed();
}

QVector3D get_pt_on_line(const QVector3D& p0, const QVector3D& n, double t)
{
	return p0 + n * t;
}

inline double sign(double v1)
{
	return v1 >= 0? 1.0 : -1.0;
}

static inline Vector3d _V(const Vector3i& v)
{
	return Vector3d(v);
}

void draw_line(const QVector3D& v1, const QVector3D& v2, const QColor& col = Qt::white)
{
	glColor3ub(col.red(), col.green(), col.blue());

	glBegin(GL_LINES);
	glVertex3d(v1.x(), v1.y(), v1.z());
	glVertex3d(v2.x(), v2.y(), v2.z());
	glEnd();
}

void draw_line(const Vector3d& v1, const Vector3d& v2 = Vector3d(), const QColor& col = Qt::white)
{
	glColor3ub(col.red(), col.green(), col.blue());

	glBegin(GL_LINES);
	glVertex3dv(v1.data);
	glVertex3dv(v2.data);
	glEnd();
}

///////////////////////////////////////////////////

const int max_trajectory_size = 200;

///////////////////////////////
/// \brief GyroData::GyroData
/// \param parent
///
///
GyroData::GyroData(QObject *parent) :
	VirtGLObject(parent)
  , m_divider_accel(5000)
  , m_divider_gyro(5000)
  , m_percent_downloaded_data(1)
  , m_showing_downloaded_data(true)
  , m_is_play(false)
  , m_current_playing_pos(0)
  , m_index(0)
  , m_is_draw_mean_sphere(true)
  , m_show_calibrated_data(true)
  , m_write_data(false)
  , m_add_to_pool(false)
  , m_show_recorded_data(false)
{
	setType(GYRODATA);

	connect(&m_timer_playing, SIGNAL(timeout()), this, SLOT(_on_timeout_playing()));
	m_timer_playing.start(100);

	m_sensorsWork = new SensorsWork();
	m_sensorsWork->moveToThread(m_sensorsWork);
	m_sensorsWork->start();

	connect(m_sensorsWork, SIGNAL(fill_data_for_calibration(const sc::StructTelemetry&)), this, SLOT(fill_data_for_calibration(const sc::StructTelemetry&)));
	connect(m_sensorsWork, SIGNAL(stop_calibration()), this, SLOT(_on_stop_calibration()));

	connect(&m_sensorsWork->calibrate_thread(), SIGNAL(send_log(QString)), this, SLOT(_on_calibrate_log(QString)));


	load_from_xml();
}

GyroData::~GyroData()
{
	save_to_xml();

	if(m_sensorsWork)
		delete m_sensorsWork;
}

QHostAddress GyroData::addr() const
{
	return m_addr;
}

ushort GyroData::port() const
{
	return m_port;
}

QString GyroData::fileName() const
{
	return m_fileName;
}

inline double get_min(const QVector3D& v, double min)
{
	double res = qMin<double>(min, v.x());
	res = qMin<double>(min, v.y());
	res = qMin<double>(min, v.z());
	return res;
}

inline double get_max(const QVector3D& v, double max)
{
	double res = qMax<double>(max, v.x());
	res = qMax<double>(max, v.y());
	res = qMax<double>(max, v.z());
	return res;
}

void normalize_vector(QVector< QVector3D> & data)
{
	double min, max;

	if(!data.size())
		return;

	min = max = data[0].x();

	foreach (QVector3D it, data) {
		min = get_min(it, min);
		max = get_max(it, max);
	}

	double delta = max - min;

	if(qFuzzyIsNull(delta))
		return;

	double delta_inv = 1.0 / delta;

	for(int i = 0; i < data.size(); i++){
		data[i] = data[i] * delta_inv;
	}
}

void GyroData::openFile(const QString fileName)
{
	if(!QFile::exists(fileName))
		return;

	QFile file(fileName);

	if(!file.open(QIODevice::ReadOnly))
		return;

	clear_data();

	m_fileName = fileName;

	QTextStream tstream(&file);

	m_downloaded_telemetries.clear();

	int ind = -1;
	while(!tstream.atEnd()){
		ind++;

		QString line = tstream.readLine();
		line = line.trimmed();
		QStringList sl = line.split(';');

		StructTelemetry st;

		double a1, a2, a3, t, g1, g2, g3, f = 100, afs = 0, fs = 0;
		long long tm = 0;

		if(!sl.size()){
			qDebug() << "data in line" << ind << "not enough:" << sl.size();
			continue;
		}
		if(sl.size() == 7){
			a1	= sl[0].toDouble();
			a2	= sl[1].toDouble();
			a3	= sl[2].toDouble();
			t	= sl[3].toDouble();
			g1	= sl[4].toDouble();
			g2	= sl[5].toDouble();
			g3	= sl[6].toDouble();
		}
		if(sl.size() >= 13){
			t	= sl[4].toDouble();
			a1	= sl[5].toDouble();
			a2	= sl[6].toDouble();
			a3	= sl[7].toDouble();
			g1	= sl[8].toDouble();
			g2	= sl[9].toDouble();
			g3	= sl[10].toDouble();
			afs	= sl[11].toDouble();
			fs	= sl[12].toDouble();
			f	= sl[13].toDouble();
		}
		if(sl.size() >= 14){
			tm = sl[14].toLongLong();
		}
		if(sl.size() >= 22){
			int c1, c2, c3;
			long long tick = 0;
			c1 = sl[15].toInt();
			c2 = sl[16].toInt();
			c3 = sl[17].toInt();
			tick = sl[18].toLongLong();
			st.compass.data = Vector3i(c1, c2, c3);
			st.compass.tick = tick;

			c1 = sl[19].toInt();
			c2 = sl[20].toInt();
			tick = sl[21].toLongLong();
			st.barometer.data = c1;
			st.barometer.temp = c2;
			st.barometer.tick = tick;
		}

		st.gyroscope.accel = (Vector3i(a1, a2, a3));
		st.gyroscope.gyro =(Vector3i(g1, g2, g3));
		st.gyroscope.temp = t;
		st.gyroscope.afs_sel = afs;
		st.gyroscope.fs_sel = fs;
		st.gyroscope.freq = f;
		st.gyroscope.tick = tm;

		m_downloaded_telemetries.push_back(st);
	}

	emit add_to_log("file loaded: \"" + m_fileName + "\"; count data: " + QString::number(m_downloaded_telemetries.size()));

	file.close();
}

void GyroData::set_address(const QHostAddress &host, ushort port)
{
	m_addr = host;
	m_port = port;

	if(!sensorsWork())
		return;

	sensorsWork()->set_address(host, port);
}

void GyroData::send_start_to_net()
{
	if(!sensorsWork())
		return;

	sensorsWork()->send_start();

	emit add_to_log("send start signal");
}

void GyroData::send_stop_to_net()
{
	if(!sensorsWork())
		return;

	sensorsWork()->send_stop();

	emit add_to_log("send stop signal");
}

double GyroData::divider_gyro() const
{
	return m_divider_gyro;
}

void GyroData::set_divider_gyro(double val)
{
	m_divider_gyro = val;
}

double GyroData::divider_accel() const
{
	return m_divider_accel;
}

void GyroData::set_divider_accel(double val)
{
	m_divider_accel = val;
}

void GyroData::set_end_pos_downloaded_data(double value)
{
	if(value < 0) value = 0;
	if(value > 100) value = 100;
	m_percent_downloaded_data = value / 100.0;
}

double GyroData::end_pos_downloaded_data() const
{
	return m_percent_downloaded_data * 100.0;
}

bool GyroData::showing_downloaded_data() const
{
	return m_showing_downloaded_data;
}

void GyroData::set_showing_downloaded_data(bool value)
{
	m_showing_downloaded_data = value;
}

bool GyroData::is_play() const
{
	return m_is_play;
}

void GyroData::play()
{
	if(!m_downloaded_telemetries.size())
		return;
	m_is_play = true;
}

void GyroData::pause()
{
	m_is_play = false;
}

void GyroData::stop()
{
	if(m_is_play){
	}
	m_is_play = false;
	m_current_playing_pos = 0;
}

double GyroData::percent_position() const
{
	int mm = m_downloaded_telemetries.size();
	if(!mm)
		return 0;
	return 100.0 * m_current_playing_pos / mm;
}

void GyroData::set_position_playback(double position)
{
	if(position < 0 || position > 100)
		return;
	int mm = m_downloaded_telemetries.size();
	double pos = mm * position / 100.0;
	m_current_playing_pos = pos;
}

double GyroData::freq_playing() const
{
	double timeout = m_timer_playing.interval();
	return 1000.0 / timeout;
}

void GyroData::set_freq_playing(double value)
{
	double timeout = 1000.0 / value;
	m_timer_playing.setInterval(timeout);
}

void GyroData::reset()
{
	clear_data();
}

void GyroData::set_init_position()
{
	if(sensorsWork()){
		sensorsWork()->set_position();
	}

	m_writed_telemetries.clear();
}

bool GyroData::calibrate_accelerometer()
{
	if(!sensorsWork())
		return false;

	if(sensorsWork()->calibrate_thread().is_progress())
		return false;

	QVector< StructTelemetry > & st = m_writed_telemetries;

	if(!m_writed_telemetries.size()){
		st = m_downloaded_telemetries;

		if(!m_downloaded_telemetries.size())
			return false;
	}

	QVector< Vector3d > data;
	foreach (StructTelemetry it, st) {
		data.push_back(it.gyroscope.accel);
	}

	return sensorsWork()->calibrate_accelerometer(data);
}

bool GyroData::calibrate_compass()
{
	if(!sensorsWork())
		return false;

	if(sensorsWork()->calibrate_thread().is_progress())
		return false;

	QVector< StructTelemetry > & st = m_writed_telemetries;

	if(!m_writed_telemetries.size()){
		st = m_downloaded_telemetries;

		if(!m_downloaded_telemetries.size())
			return false;
	}

	QVector< Vector3d > data;
	foreach (StructTelemetry it, st) {
		data.push_back(it.compass.data);
	}

	return sensorsWork()->calibrate_compass(data);
}

bool GyroData::is_draw_mean_sphere() const
{
	return m_is_draw_mean_sphere;
}

void GyroData::set_draw_mean_sphere(bool value)
{
	m_is_draw_mean_sphere = value;
}

void GyroData::set_show_calibrated_data(bool value)
{
	m_show_calibrated_data = value;
}

void GyroData::set_write_data(bool value)
{
	if(m_add_to_pool)
		return;

	m_write_data = value;
	if(value){
		m_writed_telemetries.clear();
	}
}

int GyroData::count_write_data() const
{
	return m_writed_telemetries.size() + m_pool_writed_telemetries.size();
}

void GyroData::add_to_pool(bool value)
{
	if(m_write_data)
		return;

	if(value){
		m_pool_writed_telemetries.clear();
		m_add_to_pool = true;
	}else{
		m_writed_telemetries += m_pool_writed_telemetries;
		m_pool_writed_telemetries.clear();
		m_add_to_pool = false;
	}
}

void GyroData::cancel_write()
{
	m_add_to_pool = false;
	m_pool_writed_telemetries.clear();
}

void GyroData::log_recorded_data()
{
	WriteLog::instance()->write_data("recorded_data", m_writed_telemetries);
}

void GyroData::_on_calibrate_log(const QString &data)
{
	emit add_to_log(data);
}

void GyroData::_on_stop_calibration()
{
	if(sensorsWork()->typeOfCalibrate() == SensorsWork::Accelerometer)
		m_sphereGl.generate_sphere(sensorsWork()->mean_sphere().mean_radius, m_divider_accel);
}

void GyroData::init()
{
	if(!sensorsWork())
		return;

	openFile(m_fileName);

	m_sphereGl.generate_sphere(sensorsWork()->mean_sphere().mean_radius, m_divider_accel);

	emit add_to_log("loaded. x=" + QString::number(sensorsWork()->mean_sphere().cp.x(), 'f', 3) +
			   ", y=" + QString::number(sensorsWork()->mean_sphere().cp.y(), 'f', 3) +
			   ", z=" + QString::number(sensorsWork()->mean_sphere().cp.z(), 'f', 3) +
			   "; R=" + QString::number(sensorsWork()->mean_sphere().mean_radius, 'f', 3) +
			   "; dev=" + QString::number(sensorsWork()->mean_sphere().deviation, 'f', 3));
}

void draw_axes(const Vector3d& offset = Vector3d())
{
	glPushMatrix();
	glTranslated(offset.x(), offset.y(), offset.z());

	glColor3f(1.f, 0.2f, 0.2f);
	glBegin(GL_LINES);
	glVertex3d(-1, 0, 0);
	glVertex3d(1, 0, 0);

	glVertex3d(1, 0, 0);
	glVertex3d(0.7, 0.3, 0);

	glVertex3d(1, 0, 0);
	glVertex3d(0.7, -0.3, 0);
	glEnd();

	glColor3f(0.2f, 1.f, 0.2f);
	glBegin(GL_LINES);
	glVertex3d(0, -1, 0);
	glVertex3d(0, 1, 0);

	glVertex3d(0, 1, 0);
	glVertex3d(0.3, 0.7, 0);

	glVertex3d(0, 1, 0);
	glVertex3d(-0.3, 0.7, 0);
	glEnd();

	glColor3f(0.2f, 0.2f, 1.f);
	glBegin(GL_LINES);
	glVertex3d(0, 0, -1);
	glVertex3d(0, 0, 1);

	glVertex3d(0, 0, 1);
	glVertex3d(0.3, 0, 0.7);

	glVertex3d(0, 0, 1);
	glVertex3d(-0.3, 0, 0.7);
	glEnd();

	glPopMatrix();
}

/// @test code
void draw_process_rotate(const Quaternion& q1, const Quaternion& q2, double mult)
{
	if(q1.isNull() && q2.isNull())
		return;

	Vector3d v1(1, 0, 0), v2(0, 1, 0), v3(0, 0, 1);

	const int cnt = 40;
	for(int i = 0; i < cnt; i++){
		double t = (double)i / cnt;
		double t1 = 1-t;

		Vector3d v11, v12, v13;
		Quaternion q;

		q = Quaternion::slerp(q1, q2, t);
		v11 = q.rotatedVector(v1) * mult;
		v12 = q.rotatedVector(v2) * mult;
		v13 = q.rotatedVector(v3) * mult;

		draw_line(v11, Vector3d(), QColor(32 + 223 * t1, 0, 0));
		draw_line(v12, Vector3d(), QColor(0, 32 + 223 * t1, 0));
		draw_line(v13, Vector3d(), QColor(0, 0, 32 + 223 * t1));
	}
}

void draw_static_axes()
{
	glLineWidth(4);
	draw_line(Vector3d(-10, 0, 0), Vector3d(10, 0, 0), QColor(255, 128, 128));
	draw_line(Vector3d(0, -10, 0), Vector3d(0, 10, 0), QColor(128, 255, 128));
	draw_line(Vector3d(0, 0, -10), Vector3d(0, 0, 10), QColor(128, 128, 255));
	glLineWidth(1);
}

void draw_Gaccel(const Vector3d& gaccel)
{
	glLineWidth(2);
	draw_line(Vector3d(), gaccel, Qt::yellow);
}

Vector3d SHV(const Vector3i& v)
{
	return Vector3d(v.x(), v.y(), v.z());
}

const Vector3d compass_multiply(1./99., 1./99., 1./99.);

void GyroData::draw()
{
	if(!sensorsWork())
		return;

	double div_gyro = 1.0 / m_divider_gyro;
	double div_accel = 1.0 / m_divider_accel;

	glPointSize(3);

	if(m_showing_downloaded_data){
		int count = m_percent_downloaded_data * m_downloaded_telemetries.size();

		glColor3f(0, 1, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i < count; i++) {
			Vector3d v = m_downloaded_telemetries[i].gyroscope.accel;
			if(m_show_calibrated_data)
				v -= sensorsWork()->mean_sphere().cp;

			Vector3d tmp(v * div_accel);
			glVertex3dv(tmp.data);
		}
		glEnd();

		glColor3f(1, 0, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i< count; i++) {
			Vector3d tmp(_V(m_downloaded_telemetries[i].gyroscope.gyro) * div_gyro);
			glVertex3dv(tmp.data);
		}
		glEnd();

		glColor3f(1, 0.8, 0.5);
		glBegin(GL_POINTS);
		for (int i = 0; i< count; i++) {
			Vector3d v = SHV(m_downloaded_telemetries[i].compass.data);
			v -= sensorsWork()->mean_sphere_compass().cp;
			Vector3d tmp(v * compass_multiply);
			glVertex3dv(tmp.data);
		}
		glEnd();
	}

	glLineWidth(4);


	if(sensorsWork()->is_calculated()){
//		draw_text(m_tmp_axis, "m_tmp_axis. " + QString::number(m_tmp_angle));
//		draw_line(Vector3d(), m_tmp_axis, Qt::yellow);
		draw_line(Vector3d(0, 0, -1), Vector3d(0, 0, -1) + sensorsWork()->tmp_accel * div_gyro, Qt::yellow);

		draw_Gaccel(sensorsWork()->meanGaccel * div_accel);
	}

	glPushMatrix();

	QMatrix4x4 mt = fromQuaternion(sensorsWork()->rotate_quaternion);
#ifdef QT4
	glMultMatrixd((mt.data()));
#elif defined(QT5)
	glMultMatrixf((mt.data()));
#endif

	draw_axes();

	glPopMatrix();

	draw_process_rotate(sensorsWork()->rotate_quaternion, sensorsWork()->accel_quat, 0.3);
////////////////////

//	glPushMatrix();

//	glTranslated(0.5, 0.5, 0);

//	glRotatef(m_rotate_pos.x(), 1, 0, 0);
//	glRotatef(m_rotate_pos.y(), 0, 1, 0);
//	glRotatef(m_rotate_pos.z(), 0, 0, 1);
//	draw_axes();

//	glPopMatrix();

//////////////////

	glLineWidth(1);

	draw_static_axes();

/// @test code
//	{
//		Vector3d vv(0, 2, 0);
//		Quaternion q1 = Quaternion::fromAxisAndAngle(Vector3d(0, 0.5, 1), 132);
//		Quaternion q2 = Quaternion::fromAxisAndAngle(Vector3d(0, 0.5, 1), -45);
//		const int cnt = 100;
//		for(int i = 0; i < cnt; i++){
//			double t = (double)i/cnt;
//			Quaternion q = Quaternion::slerp(q1, q2, t);
//			Vector3d vvv = q.rotatedVector(vv);
//			QColor c(128 + 128 * t, 255 * t, 64 * t);
//			draw_line(Vector3d(), vvv, c);
//		}
//	}

	glPointSize(4);

	if(sensorsWork()->telemetries.size()){

		Vector3d tmp(_V(sensorsWork()->telemetries[0].gyroscope.gyro) * div_gyro);

		glColor3f(1, 0.5, 0);
		glBegin(GL_POINTS);
			glVertex3dv(tmp.data);
		glEnd();

		glLineWidth(3);
		draw_line(Vector3d(), tmp, QColor(255, 128, 0));

		glLineWidth(1);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < sensorsWork()->telemetries.size(); i++){
			StructTelemetry& st = sensorsWork()->telemetries[i];
			float dd = (float)(sensorsWork()->telemetries.size() - i) / sensorsWork()->telemetries.size();
			glColor3f(1 * dd, 0.5 * dd, 0);

			tmp = _V(st.gyroscope.gyro) * div_gyro;

			glVertex3dv(tmp.data);
		}
		glEnd();

		tmp = sensorsWork()->mean_accel;
		if(!m_show_calibrated_data)		/// there because in analize_telemetry thise operation already made
			tmp += sensorsWork()->mean_sphere().cp;
		tmp *= div_accel;

		glPushMatrix();

		draw_line(Vector3d(), tmp, Qt::green);
		draw_text(tmp, "accel. " + QString::number(sensorsWork()->telemetries[0].gyroscope.accel.length()));

//		tmp = sensorsWork()->rotate_quaternion.rotatedVector(tmp);
//		draw_line(Vector3d(), tmp, QColor(200, 255, 100));

//		glLineWidth(2);
//		tmp = Vector3d(-tmp.x(), -tmp.y(), tmp.z());
//		draw_line(Vector3d(), tmp, QColor(230, 155, 64));

		{
			Vector3d cmp = SHV(sensorsWork()->telemetries[0].compass.data);
			cmp -= sensorsWork()->mean_sphere_compass().cp;
			cmp = cmp * compass_multiply;
			glLineWidth(3);
			draw_line(cmp, Vector3d(), QColor(128, 100, 64));
			draw_text(cmp, "compass", QColor(128, 100, 64));

			Vector3d cmpXY = cmp;
			cmpXY.setZ(0);

			cmp = sensorsWork()->rotate_quaternion.rotatedVector(cmp);
			draw_line(cmp, Vector3d(), QColor(128, 100, 255));
			draw_text(cmp, "compass2", QColor(128, 100, 64));

			cmpXY.normalize();
			draw_line(cmpXY, Vector3d(), QColor(228, 100, 100));
		}

		glLineWidth(1);

		if(!m_show_calibrated_data){
			glBegin(GL_LINE_STRIP);
			for (int i = 0; i < sensorsWork()->telemetries.size(); i++){
				StructTelemetry& st = sensorsWork()->telemetries[i];
				float dd = (float)(sensorsWork()->telemetries.size() - i) / sensorsWork()->telemetries.size();
				glColor3f(0.5 * dd, 1 * dd, 0);

				tmp = _V(st.gyroscope.accel) + sensorsWork()->mean_sphere().cp;
				tmp *= div_accel;

				glVertex3dv(tmp.data);
			}
			glEnd();

			glBegin(GL_LINE_STRIP);
			for (int i = 0; i < sensorsWork()->telemetries.size(); i++){
				StructTelemetry& st = sensorsWork()->telemetries[i];
				float dd = (float)(sensorsWork()->telemetries.size() - i) / sensorsWork()->telemetries.size();
				glColor3f(0.3f, 1 * dd, 0.5f * dd);

				tmp = _V(st.compass.data);
				tmp = tmp * compass_multiply;

				glVertex3dv(tmp.data);
			}
			glEnd();
		}else{
			glBegin(GL_LINE_STRIP);
			for (int i = 0; i < sensorsWork()->telemetries.size(); i++){
				StructTelemetry& st = sensorsWork()->telemetries[i];
				float dd = (float)(sensorsWork()->telemetries.size() - i) / sensorsWork()->telemetries.size();
				glColor3f(0.5 * dd, 1 * dd, 0);

				tmp = _V(st.gyroscope.accel) * div_accel;

				glVertex3dv(tmp.data);
			}
			glEnd();

			glBegin(GL_LINE_STRIP);
			for (int i = 0; i < sensorsWork()->telemetries.size(); i++){
				StructTelemetry& st = sensorsWork()->telemetries[i];
				float dd = (float)(sensorsWork()->telemetries.size() - i) / sensorsWork()->telemetries.size();
				glColor3f(0.3f, 1 * dd, 0.5f * dd);

				tmp = _V(st.compass.data) - sensorsWork()->mean_sphere_compass().cp;
				tmp = tmp * compass_multiply;

				glVertex3dv(tmp.data);
			}
			glEnd();
		}

		glPopMatrix();
	}

	if(m_is_draw_mean_sphere){
		m_sphereGl.draw_sphere(sensorsWork()->mean_sphere().cp, m_divider_accel);
	}

	if(m_show_recorded_data){
		draw_recored_data();
	}

	glColor3f(1, 0.2, 0);
	glPointSize(3);
	if((sensorsWork()->is_calculated()) && m_trajectory.size()){
		glBegin(GL_POINTS);
		foreach (Vector3f v, m_trajectory) {
			glVertex3fv(v.data);
		}
		glEnd();

		while(m_trajectory.size() > max_trajectory_size){
			m_trajectory.pop_front();
		}
	}

	calc_parameters();
}

void GyroData::tick()
{
	emit set_text("sphere_radius", QString::number(sensorsWork()->mean_sphere().mean_radius, 'f', 1));
	emit set_text("sphere_cp", sensorsWork()->mean_sphere().cp);
	emit set_text("sphere_dev", QString::number(sensorsWork()->mean_sphere().deviation, 'f', 1));
}

QVector3D GyroData::position() const
{
	return QVector3D();
}

void GyroData::clear_data()
{
	m_percent_downloaded_data = 1;
	m_index = 0;
	m_trajectory.clear();
	m_write_data = false;
	m_writed_telemetries.clear();
}

void GyroData::_on_timeout_playing()
{
	if(m_is_play && (m_downloaded_telemetries.size())){

		if(m_current_playing_pos >= m_downloaded_telemetries.size()){
			m_current_playing_pos = 0;
			set_init_position();
		}

		StructTelemetry st = m_downloaded_telemetries[m_current_playing_pos];

		st = sensorsWork()->analyze_telemetry(st);

		m_current_playing_pos++;
		m_index++;
	}
}

void GyroData::load_from_xml()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir + xml_config;

	SimpleXML sxml(config_file, SimpleXML::READ);

	if(!sxml.isLoaded())
		return;

	set_end_pos_downloaded_data((double)sxml["end_position"] * 100.0);
	m_divider_accel = sxml["divider_accel"];
	m_divider_gyro = sxml["divider_gyro"];
	m_fileName = (QString)sxml["filename"];
	m_showing_downloaded_data = sxml["showing_downloaded_data"];
	m_addr = QHostAddress((QString)sxml["ip"]);
	ushort port = sxml["port"];

	if(m_addr.isNull()){
		m_addr = QHostAddress("192.168.0.200");
	}
	if(port)
		m_port = port;

	double freq = sxml["freq_playing"];
	if(freq){
		set_freq_playing(freq);
	}

	m_is_draw_mean_sphere = sxml["draw_mean_sphere"];

	m_show_calibrated_data = sxml["show_calibrated_data"];

	m_show_recorded_data = sxml["show_recorded_data"];

	if(!QFile::exists(m_fileName))
		m_fileName.clear();

	if(sensorsWork()){
		sensorsWork()->set_address(m_addr, m_port);
	}
}

void GyroData::save_to_xml()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir;

	QDir dir(QDir::homePath());

	//if(!dir.exists(config_file))
	dir.mkdir(config_file);

	config_file += xml_config;

	SimpleXML sxml(config_file, SimpleXML::WRITE);

	sxml << "end_position" << m_percent_downloaded_data;
	sxml << "divider_accel" << m_divider_accel;
	sxml << "divider_gyro" << m_divider_gyro;
	sxml << "filename" << m_fileName;
	sxml << "showing_downloaded_data" << m_showing_downloaded_data;
	sxml << "ip" << m_addr.toString();
	sxml << "port" << m_port;

	double freq = freq_playing();
	sxml << "freq_playing" << freq;
	sxml << "draw_mean_sphere" << m_is_draw_mean_sphere;

	sxml << "show_calibrated_data" << m_show_calibrated_data;
	sxml << "show_recorded_data" << m_show_recorded_data;
}

void GyroData::reset_trajectory()
{
	m_trajectory.clear();
}

void GyroData::show_recorded_data(bool value)
{
	m_show_recorded_data = value;
}

SensorsWork *GyroData::sensorsWork()
{
	return m_sensorsWork;
}

void GyroData::draw_text(const Vector3d &v, const QString &text, const QColor& col)
{
	QGLWidget *w = dynamic_cast< QGLWidget* >(parent());
	if(w){
		glColor3f(col.redF(), col.greenF(), col.blueF());
		w->renderText(v.x(), v.y(), v.z(), text, QFont("Helvetica [Cronyx]", 14));
	}
}

void GyroData::draw_recored_data()
{	
	glColor3f(1, 0.5, 0.3);
	glBegin(GL_POINTS);
	foreach (StructTelemetry st, m_writed_telemetries) {
		Vector3d v = st.gyroscope.accel;
		v *= 1. / m_divider_accel;
		glVertex3dv(v.data);
	}
	glEnd();

	if(!sensorsWork())
		return;

	glColor3f(0.5, 1, 0.3);
	glBegin(GL_POINTS);
	foreach (StructTelemetry st, m_writed_telemetries) {
		Vector3d v = st.compass.data;
		v -= sensorsWork()->mean_sphere_compass().cp;
		v = v * compass_multiply;
		glVertex3dv(v.data);
	}
	glEnd();
}

void GyroData::fill_data_for_calibration(const StructTelemetry &st)
{
	if(!sensorsWork())
		return;

	if(sensorsWork()->is_calculated() && m_write_data){
		m_writed_telemetries.push_back(st);
	}
	if(sensorsWork()->is_calculated() && m_add_to_pool){
		m_pool_writed_telemetries.push_back(st);
	}
}

void GyroData::calc_parameters()
{
	Vector3d v1(1, 0, 0), v2, v3(0, 1, 0);
	v2 = sensorsWork()->rotate_quaternion.rotatedVector(v1);
	double an = common_::rad2angle(atan2(v2.z(), v2.x()));
	set_text("tangaj", QString::number(an, 'f', 1));

	v2 = sensorsWork()->rotate_quaternion.rotatedVector(v3);
	an = common_::rad2angle(atan2(v2.z(), v2.y()));
	set_text("bank", QString::number(an, 'f', 1));
}
