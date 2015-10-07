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

///////////////////////////////
Q_DECLARE_METATYPE(Vector3i)

const QString xml_config("gyro.xml");
const QString xml_calibrate("calibrate.xml");

///////////////////////////////

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

inline Vector3i rshift(const Vector3i& val, int shift)
{
	Vector3i res;
	FOREACH(i, 3, res.data[i] = val.data[i] >> shift);
	return res;
}

void parse_quaternsion(const QQuaternion& qa, QVector3D& axes, double& angle)
{
	QQuaternion q = qa.normalized();

	double qw = q.scalar();
	if(qFuzzyCompare(fabs(qw), 1)){
		angle = 0;
		axes = QVector3D(0, 0, 1);
		return;
	}

	double div = 1.0 / sqrt(1 - qw * qw);
	/*
	 *  angle = 2 * acos(qw)
	 *	x = qx / sqrt(1-qw*qw)
	 *	y = qy / sqrt(1-qw*qw)
	 *	z = qz / sqrt(1-qw*qw)
	 */

	angle = 2 * acos(q.scalar());
	angle *= 180.0 / M_PI;
	axes.setX(q.x() * div);
	axes.setX(q.y() * div);
	axes.setX(q.z() * div);
}

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

///////////////////////////////////////////////////

const int max_trajectory_size = 2000;

///////////////////////////////
/// \brief GyroData::GyroData
/// \param parent
///
///
GyroData::GyroData(QObject *parent) :
	VirtGLObject(parent)
  , m_socket(0)
  , m_divider_accel(5000)
  , m_divider_gyro(5000)
  , m_percent_downloaded_data(1)
  , m_showing_downloaded_data(true)
  , m_is_play(false)
  , m_current_playing_pos(0)
  , m_is_calc_offset_gyro(false)
  , m_count_gyro_offset_data(0)
  , m_is_calculated(false)
  , m_tmp_angle(0)
  , m_port(7777)
  , m_addr(QHostAddress("192.168.0.200"))
  , m_part_of_time(5.0/1000.0)	/// default: (5/1000)
  , m_past_tick(0)
  , m_first_tick(0)
  , m_index(0)
  , m_is_draw_mean_sphere(true)
  , m_show_calibrated_data(true)
  , m_write_data(false)
  , m_threshold_correction(3)
  , m_multiply_correction(0.1)
  , m_threshold_accel(0.07)
{
	setType(GYRODATA);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_timeout()));
	m_timer.start(300);

	connect(&m_timer_playing, SIGNAL(timeout()), this, SLOT(on_timeout_playing()));
	m_timer_playing.start(100);

	connect(&m_timer_calibrate, SIGNAL(timeout()), this, SLOT(on_timeout_calibrate()));
	m_timer_calibrate.setInterval(300);

	m_time_waiting_telemetry.start();

	connect(&m_calibrate, SIGNAL(send_log(QString)), this, SLOT(on_calibrate_log(QString)));

	load_from_xml();
}

GyroData::~GyroData()
{
	if(m_socket){
		m_socket->abort();
	}
	delete m_socket;

	save_to_xml();
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

	int ind = 0;
	while(!tstream.atEnd()){
		QString line = tstream.readLine();
		line = line.trimmed();
		QStringList sl = line.split(';');

		StructTelemetry st;

		double a1, a2, a3, t, g1, g2, g3, f = 100, afs = 0, fs = 0;
		long long tm = 0;

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
		if(!sl.size()){
			qDebug() << "data in line" << ind << "not enough:" << sl.size();
			continue;
		}

		st.accel = (Vector3i(a1, a2, a3));
		st.gyro =(Vector3i(g1, g2, g3));
		st.temp = t;
		st.afs_sel = afs;
		st.fs_sel = fs;
		st.freq = f;
		st.tick = tm;

		m_downloaded_telemetries.push_back(st);

		ind++;
	}

	emit add_to_log("file loaded: \"" + m_fileName + "\"; count data: " + QString::number(m_downloaded_telemetries.size()));

	file.close();
}

void GyroData::send_start_to_net(const QHostAddress &host, ushort port)
{
	if(!m_socket){
		m_socket = new QUdpSocket;
		connect(m_socket, SIGNAL(readyRead()), this, SLOT(on_readyRead()));
	}
	QByteArray data("START");
	m_socket->writeDatagram(data, host, port);

	m_addr = host;
	m_port = port;
	emit add_to_log("send start signal");
}

void GyroData::send_stop_to_net(const QHostAddress &host, ushort port)
{
	if(!m_socket){
		m_socket = new QUdpSocket;
		connect(m_socket, SIGNAL(readyRead()), this, SLOT(on_readyRead()));
	}
	QByteArray data("STOP");
	m_socket->writeDatagram(data, host, port);

	m_addr = host;
	m_port = port;

	emit add_to_log("send stop signal");
}

QHostAddress GyroData::addr() const
{
	return m_addr;
}

ushort GyroData::port() const
{
	return m_port;
}

bool GyroData::is_available_telemetry() const
{
	return m_time_waiting_telemetry.elapsed() < max_delay_for_data;
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
		m_telemetries.clear();
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

void GyroData::start_calc_offset_gyro()
{
	m_is_calc_offset_gyro = true;
	m_is_calculated = false;
	m_offset_gyro = Vector3d();
	m_count_gyro_offset_data = 0;
	m_rotate_pos = Vector3d();
	m_translate_pos = Vector3d();
	m_translate_speed = Vector3f();
	m_mean_Gaccel = Vector3d();
	m_rotate_quaternion = Quaternion();
	m_len_Gaccel = 0;
}

void GyroData::stop_calc_offset_gyro()
{
	m_is_calc_offset_gyro = false;
	if(m_count_gyro_offset_data){
		m_offset_gyro *= 1.0/m_count_gyro_offset_data;
		m_mean_Gaccel *= 1.0/m_count_gyro_offset_data;

		m_len_Gaccel = m_mean_Gaccel.length();

		m_tmp_accel = m_mean_Gaccel;

		m_rotate_pos = Vector3d();
		m_rotate_quaternion = Quaternion();
		m_translate_pos = Vector3d();
		m_is_calculated = true;

		Vector3d v = m_mean_Gaccel;
		emit add_to_log("offset values.  gyroscope: " + QString::number(m_offset_gyro.x(), 'f', 3) + ", " +
						QString::number(m_offset_gyro.y(), 'f', 3) + ", " +
						QString::number(m_offset_gyro.z(), 'f', 3) + "; accelerometer: " +
						QString::number(v.x(), 'f', 3) + ", " +
						QString::number(v.y(), 'f', 3) + ", " +
						QString::number(v.z(), 'f', 3));
	}
}

int GyroData::count_gyro_offset_data() const
{
	return m_count_gyro_offset_data;
}

void GyroData::reset()
{
	clear_data();
}

void GyroData::set_init_position()
{
	m_rotate_pos = Vector3d();
	m_translate_pos = Vector3d();
	m_translate_speed = Vector3d();
	m_tmp_axis = Vector3f(1, 0, 0);
	m_tmp_angle = 0;
	m_mean_accel = Vector3d();
	m_past_tick = 0;
	m_first_tick = 0;
	m_rotate_quaternion = Quaternion();
	m_writed_telemetries.clear();
}

bool GyroData::calibrate()
{
	if(m_calibrate.is_progress())
		return false;

	if(m_writed_telemetries.size()){
		m_calibrate.set_parameters(&m_writed_telemetries);
	}else{
		m_calibrate.set_parameters(&m_downloaded_telemetries);

		if(!m_downloaded_telemetries.size())
			return false;
	}

	m_timer_calibrate.start();

	QThreadPool::globalInstance()->start(new CalibrateAccelerometerRunnable(&m_calibrate));

	return true;
}

const CalibrateAccelerometer &GyroData::calibrateAccelerometer() const
{
	return m_calibrate;
}

bool GyroData::is_draw_mean_sphere() const
{
	return m_is_draw_mean_sphere;
}

void GyroData::set_draw_mean_sphere(bool value)
{
	m_is_draw_mean_sphere = value;
}

void GyroData::reset_mean_sphere()
{
	m_sphere.reset();
}

void GyroData::set_show_calibrated_data(bool value)
{
	m_show_calibrated_data = value;
}

void GyroData::set_write_data(bool value)
{
	m_write_data = value;
	if(value){
		m_writed_telemetries.clear();
	}
}

void GyroData::on_timeout()
{
	if(m_telemetries.size() > 3){
		m_telemetries.pop_back();
	}
}

void GyroData::on_timeout_calibrate()
{
	if(m_calibrate.is_done()){
		m_timer_calibrate.stop();
		m_sphere = m_calibrate.result();

		emit add_to_log("evaluate. x=" + QString::number(m_sphere.cp.x(), 'f', 3) +
				   ", y=" + QString::number(m_sphere.cp.y(), 'f', 3) +
				   ", z=" + QString::number(m_sphere.cp.z(), 'f', 3) +
				   "; R=" + QString::number(m_sphere.mean_radius, 'f', 3) +
				   "; dev=" + QString::number(m_sphere.deviation, 'f', 3));

		init_sphere();

		save_calibrate();
	}
}

inline Vector3d QV3V3(const QVector3D& v)
{
	return Vector3d(v.x(), v.y(), v.z());
}

inline QVector3D V3QV3(const Vector3d& v)
{
	return QVector3D(v.x(), v.y(), v.z());
}

void GyroData::on_readyRead()
{
	if(!m_socket)
		return;

	QByteArray data;
	while(m_socket->hasPendingDatagrams()){
		qint64 size = m_socket->pendingDatagramSize();
		data.resize(size);
		m_socket->readDatagram(data.data(), size);

		tryParseData(data);
	}
}

void GyroData::on_calibrate_log(const QString &data)
{
	emit add_to_log(data);
}

void GyroData::init()
{
	openFile(m_fileName);

	load_calibrate();

	init_sphere();

	emit add_to_log("loaded. x=" + QString::number(m_sphere.cp.x(), 'f', 3) +
			   ", y=" + QString::number(m_sphere.cp.y(), 'f', 3) +
			   ", z=" + QString::number(m_sphere.cp.z(), 'f', 3) +
			   "; R=" + QString::number(m_sphere.mean_radius, 'f', 3) +
			   "; dev=" + QString::number(m_sphere.deviation, 'f', 3));
}

void draw_axes(const Vector3d& offset = Vector3d())
{
	glPushMatrix();
	glTranslated(offset.x(), offset.y(), offset.z());

	glColor3f(1.f, 0.2f, 0.2f);
	glBegin(GL_LINES);
	glVertex3d(0, 0, 0);
	glVertex3d(1, 0, 0);

	glVertex3d(1, 0, 0);
	glVertex3d(0.7, 0.3, 0);

	glVertex3d(1, 0, 0);
	glVertex3d(0.7, -0.3, 0);
	glEnd();

	glColor3f(0.2f, 1.f, 0.2f);
	glBegin(GL_LINES);
	glVertex3d(0, 0, 0);
	glVertex3d(0, 1, 0);

	glVertex3d(0, 1, 0);
	glVertex3d(0.3, 0.7, 0);

	glVertex3d(0, 1, 0);
	glVertex3d(-0.3, 0.7, 0);
	glEnd();

	glColor3f(0.2f, 0.2f, 1.f);
	glBegin(GL_LINES);
	glVertex3d(0, 0, 0);
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

void GyroData::draw()
{
	double div_gyro = 1.0 / m_divider_gyro;
	double div_accel = 1.0 / m_divider_accel;

	glPointSize(3);

	if(m_showing_downloaded_data){
		int count = m_percent_downloaded_data * m_downloaded_telemetries.size();

		glColor3f(0, 1, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i < count; i++) {
			Vector3d v = m_downloaded_telemetries[i].accel;
			if(m_show_calibrated_data)
				v -= m_sphere.cp;

			Vector3d tmp(v * div_accel);
			glVertex3dv(tmp.data);
		}
		glEnd();

		glColor3f(1, 0, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i< count; i++) {
			Vector3d tmp(_V(m_downloaded_telemetries[i].gyro) * div_gyro);
			glVertex3dv(tmp.data);
		}
		glEnd();
	}

	glLineWidth(4);


	if(m_is_calculated){
//		draw_text(m_tmp_axis, "m_tmp_axis. " + QString::number(m_tmp_angle));
//		draw_line(Vector3d(), m_tmp_axis, Qt::yellow);
		draw_line(Vector3d(), m_tmp_accel * div_gyro, Qt::yellow);
		draw_line(Vector3d(), m_speed_accel * div_gyro, Qt::gray);
	}

	glPushMatrix();

	QMatrix4x4 mt = fromQuaternion(m_rotate_quaternion);
#ifdef QT4
	glMultMatrixd((mt.data()));
#elif defined(QT5)
	glMultMatrixf((mt.data()));
#endif

	draw_axes();

	glPopMatrix();

	draw_process_rotate(m_rotate_quaternion, m_accel_quat, 0.3);
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

	if(m_telemetries.size()){

		Vector3d tmp(_V(m_telemetries[0].gyro) * div_gyro);

		glColor3f(1, 0.5, 0);
		glBegin(GL_POINTS);
			glVertex3dv(tmp.data);
		glEnd();

		glLineWidth(3);
		draw_line(Vector3d(), tmp, QColor(255, 128, 0));

		glLineWidth(1);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < m_telemetries.size(); i++){
			StructTelemetry& st = m_telemetries[i];
			float dd = (float)(m_telemetries.size() - i) / m_telemetries.size();
			glColor3f(1 * dd, 0.5 * dd, 0);

			tmp = _V(st.gyro) * div_gyro;

			glVertex3dv(tmp.data);
		}
		glEnd();

		tmp = m_mean_accel;
		if(!m_show_calibrated_data)		/// there because in analize_telemetry thise operation already made
			tmp += m_sphere.cp;
		tmp *= div_accel;

		glPushMatrix();

		draw_line(Vector3d(), tmp, Qt::green);
		draw_text(tmp, "accel. " + QString::number(m_telemetries[0].accel.length()));

		glLineWidth(1);
		glBegin(GL_LINE_STRIP);

		if(!m_show_calibrated_data){
			for (int i = 0; i < m_telemetries.size(); i++){
				StructTelemetry& st = m_telemetries[i];
				float dd = (float)(m_telemetries.size() - i) / m_telemetries.size();
				glColor3f(0.5 * dd, 1 * dd, 0);

				tmp = _V(st.accel) + m_sphere.cp;
				tmp *= div_accel;

				glVertex3dv(tmp.data);
			}
		}else{
			for (int i = 0; i < m_telemetries.size(); i++){
				StructTelemetry& st = m_telemetries[i];
				float dd = (float)(m_telemetries.size() - i) / m_telemetries.size();
				glColor3f(0.5 * dd, 1 * dd, 0);

				tmp = _V(st.accel) * div_accel;

				glVertex3dv(tmp.data);
			}
		}
		glEnd();

		glPopMatrix();
	}

	if(m_is_draw_mean_sphere){
		draw_sphere();
	}

	glColor3f(1, 0.2, 0);
	glPointSize(3);
	if((m_is_calculated) && m_trajectory.size()){
		glBegin(GL_POINTS);
		foreach (Vector3f v, m_trajectory) {
			glVertex3fv(v.data);
		}
		glEnd();
	}
}

void GyroData::tick()
{
}

QVector3D GyroData::position() const
{
	return QVector3D();
}

void GyroData::calc_offsets(Vector3i &gyro, Vector3i &accel)
{
	if(m_is_calc_offset_gyro){
		m_offset_gyro += gyro;
		m_mean_Gaccel += accel;

		m_count_gyro_offset_data++;
	}else{
//		gyro -= m_offset_gyro;
//		accel -= m_offset_accel;
	}
}

void GyroData::clear_data()
{
	m_is_calculated = false;
	m_count_gyro_offset_data = 0;
	m_percent_downloaded_data = 1;
	m_offset_gyro = Vector3d();
	m_rotate_pos = Vector3d();
	m_tmp_axis = Vector3f(1, 0, 0);
	m_tmp_angle = 0;
	m_mean_accel = Vector3d();
	m_past_tick = 0;
	m_first_tick = 0;
	m_index = 0;
	m_trajectory.clear();
	m_translate_pos = Vector3d();
	m_translate_speed = Vector3d();
	m_rotate_quaternion = Quaternion();
	m_write_data = false;
	m_writed_telemetries.clear();
}

void GyroData::on_timeout_playing()
{
	if(m_is_play && (m_downloaded_telemetries.size())){

		if(m_current_playing_pos >= m_downloaded_telemetries.size()){
			m_current_playing_pos = 0;
			set_init_position();
		}

		StructTelemetry st = m_downloaded_telemetries[m_current_playing_pos];

		if(st.tick && m_first_tick){
			long long tick = st.tick - m_first_tick;
			qDebug() << m_index << tick - m_past_tick << st.tick << m_first_tick + m_past_tick;
			m_part_of_time = (double)(tick - m_past_tick) / 1e+3;
			m_past_tick = tick;
		}else{
			if(st.tick){
				m_first_tick = st.tick;
				m_past_tick = st.tick - m_first_tick;
			}
		}

		analyze_telemetry(st);

		m_current_playing_pos++;
		m_index++;
	}

	while(m_telemetries.size() >= max_count_telemetry){
		m_telemetries.pop_back();
	}
}

void remove_lowbits(Vector3i& v, int bits)
{
	FOREACH(i, Vector3i::count, v.data[i] = (v.data[i] >> bits) << bits);
}

void GyroData::tryParseData(const QByteArray &data)
{
	if(!m_tick_telemetry.isValid() || m_tick_telemetry.hasExpired(max_delay_for_data)){
		m_tick_telemetry.restart();
	}

	StructTelemetry st;

	QDataStream stream(data);
	st.read_from(stream);

//	remove_lowbits(st.accel, 3);
//	remove_lowbits(st.gyro, 4);
//	qint64 tick_delta = m_tick_telemetry.nsecsElapsed();
//	double part_of_time = tick_delta / 1e+9;
//	m_part_of_time = part_of_time;

//	m_tick_telemetry.restart();
	if(st.tick && m_first_tick){
		long long tick = st.tick - m_first_tick;
		qDebug() << m_index << tick - m_past_tick << st.tick << m_first_tick + m_past_tick;
		m_part_of_time = (double)(tick - m_past_tick) / 1e+3;
		m_past_tick = tick;
	}else{
		if(st.tick){
			m_first_tick = st.tick;
			m_past_tick = st.tick - m_first_tick;
		}
	}

	analyze_telemetry(st);

	if(m_is_calculated && m_write_data){
		m_writed_telemetries.push_back(st);
	}

	WriteLog::instance()->add_data("gyro", st);

	m_time_waiting_telemetry.restart();

	m_index++;

	while(m_telemetries.size() >= max_count_telemetry){
		m_telemetries.pop_back();
	}
}

const int min_threshold_accel = 200;

static inline Quaternion fromAnglesAxes(const Vector3d& angles)
{
	Quaternion/* qX, qY, qZ*/ qres;
	double aspeed = angles.length();
	if(!sc::fIsNull(aspeed)){
		Vector3d axis = angles.normalized();
		qres = Quaternion::fromAxisAndAngle(axis, aspeed);
	}
//	qX = Quaternion::fromAxisAndAngle(Vector3d(1, 0, 0), angles.x());
//	qY = Quaternion::fromAxisAndAngle(Vector3d(0, 1, 0), angles.y());
//	qZ = Quaternion::fromAxisAndAngle(Vector3d(0, 0, 1), angles.z());
//	qres = qX * qY * qZ;
	return qres;
}

static inline QQuaternion fromAnglesAxesTst(const Vector3d& angles)
{
	QQuaternion qX, qY, qZ, qres;
	qX = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), angles.x());
	qY = QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), angles.y());
	qZ = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), angles.z());
	qres = qX * qY * qZ;
	return qres;
}

double rnd_a(){
	return -360.0 + (double)(rand()) / RAND_MAX * 720.0;
}

/// @test code
bool test_data()
{
	srand(QDateTime::currentMSecsSinceEpoch());
	double ll = 0;
	for(int i = 0; i < 1000; i++){
		Vector3d v(rnd_a(), rnd_a(), rnd_a());
		Quaternion q1 = fromAnglesAxes(v);
		QQuaternion qq1 = fromAnglesAxesTst(v);
		double l = qq1.lengthSquared() - q1.lengthSquared();
		ll += l * l;
		qDebug() << qq1 << q1 << ll;
	}

	Quaternion q1 = Quaternion::fromAxisAndAngle(rnd_a(), rnd_a(), rnd_a(), rnd_a());
	Quaternion q2 = Quaternion::fromAxisAndAngle(rnd_a(), rnd_a(), rnd_a(), rnd_a());

	qDebug() << "slerp";
	qDebug() << q1 << q2;

	for(int i = 0; i < 1000; i++){
		Quaternion q = Quaternion::slerp(q1, q2, (double)i / 1000.0);
		qDebug() << (double)i / 1000.0 << "q=" << q;
	}
	qDebug() << q1 << q2;

	return true;
}

/// @test code
///const bool test_fl = test_data();

void GyroData::analyze_telemetry(StructTelemetry &st)
{
	st.accel -= m_sphere.cp;

	emit get_data("gyro", st.gyro);
	emit get_data("accel", st.accel);

	m_kalman[0].set_zk(st.accel.x());
	m_kalman[1].set_zk(st.accel.y());
	m_kalman[2].set_zk(st.accel.z());

	m_kalman[3].set_zk(st.gyro.x());
	m_kalman[4].set_zk(st.gyro.y());
	m_kalman[5].set_zk(st.gyro.z());

	Vector3d kav(Vector3d(m_kalman[0].xk, m_kalman[1].xk, m_kalman[2].xk));
	Vector3d kgv(Vector3d(m_kalman[3].xk, m_kalman[4].xk, m_kalman[5].xk));

	st.accel = kav;
	st.gyro = kgv;

	emit get_data("kalman_accel", kav);
	emit get_data("kalman_gyro", kgv);

	calc_offsets(st.gyro, st.accel);

	if(m_mean_accel.isNull())
		m_mean_accel = st.accel;
	m_mean_accel = m_mean_accel * 0.9 + Vector3d(st.accel) * 0.1;

	if(!m_is_calc_offset_gyro && m_is_calculated){
		//m_rotate_pos -= (st.angular_speed(m_offset_gyro) * m_part_of_time);
		Vector3d rotate_speed = (st.angular_speed(m_offset_gyro) * m_part_of_time);
		rotate_speed = rotate_speed.inv();

		Quaternion quat_speed = fromAnglesAxes(rotate_speed);
		m_rotate_quaternion *= quat_speed;

		Vector3d accel_mg = m_rotate_quaternion.conj().rotatedVector(m_mean_accel);

		if(!m_prev_accel.isNull()){
			m_tmp_accel = accel_mg - m_prev_accel;
			m_speed_accel += m_tmp_accel;
		}
		m_prev_accel = accel_mg;

		Vector3d vga = m_rotate_quaternion.rotatedVector(m_mean_Gaccel).normalized();
		Vector3d va = m_mean_accel.normalized();
		Vector3d ax = Vector3d::cross(va, vga).normalized();
		double an = Vector3d::dot(vga, va);
		an = rad2angle(acos(an));
		m_accel_quat = Quaternion::fromAxisAndAngle(ax, -an) * m_rotate_quaternion;
		m_accel_quat.normalize();

		if(fabs(an) > m_threshold_correction && m_tmp_accel.length() < m_threshold_accel * m_sphere.mean_radius)
			m_rotate_quaternion = Quaternion::slerp(m_rotate_quaternion, m_accel_quat, m_multiply_correction);
	}

	m_telemetries.push_front(st);
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

	double v1 = sxml["threshold_correction"];
	if(v1)
		m_threshold_correction = v1;
	v1 = sxml["multiply_correction"];
	if(v1)
		m_multiply_correction = v1;

	if(!QFile::exists(m_fileName))
		m_fileName.clear();
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

	sxml << "threshold_correction" << m_threshold_correction;
	sxml << "multiply_correction" << m_multiply_correction;
	sxml << "show_calibrated_data" << m_show_calibrated_data;
}

void GyroData::load_calibrate()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir + xml_calibrate;

	SimpleXML sxml(config_file, SimpleXML::READ);

	if(!sxml.isLoaded())
		return;

	Vector3d v;
	SimpleXMLNode node = sxml["acceleration"];
	v.setX((double)node["x_corr"]);
	v.setY(node["y_corr"]);
	v.setZ(node["z_corr"]);
	m_sphere.cp = v;
	m_sphere.mean_radius = node["mean_radius"];
	m_sphere.deviation = node["deviation"];
	if(!node["threshold"].empty())
		m_threshold_accel = node["threshold"];

	node = sxml["gyroscope"];
	v.setX(node["x_corr"]);
	v.setY(node["y_corr"]);
	v.setZ(node["z_corr"]);

	bool fl = false;
	if(!v.isNull()){
		m_offset_gyro = v;
		fl = true;
	}

	node = sxml["mean_g_accel"];
	if(!node.empty()){
		v.setX(node["x_corr"]);
		v.setY(node["y_corr"]);
		v.setZ(node["z_corr"]);
		m_mean_Gaccel = v;
	}else
		fl = false;

	if(fl){
		m_is_calculated = true;
		m_is_calc_offset_gyro = false;
	}
}

void GyroData::save_calibrate()
{
	if(m_sphere.isNull())
		return;

	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir;

	QDir dir(QDir::homePath());

	//if(!dir.exists(config_file))
	dir.mkdir(config_file);

	config_file += xml_calibrate;

	SimpleXML sxml(config_file, SimpleXML::WRITE);

	SimpleXMLNode node = sxml["acceleration"];
	node << "x_corr" <<  m_sphere.cp.x() << "y_corr" <<  m_sphere.cp.y() << "z_corr" << m_sphere.cp.z();
	node << "mean_radius" << m_sphere.mean_radius;
	node << "deviation" << m_sphere.deviation;
	node << "threshold" << m_threshold_accel;

	if(!m_offset_gyro.isNull()){
		node = sxml["gyroscope"];
		node << "x_corr" << m_offset_gyro.x() <<
		"y_corr" << m_offset_gyro.y() <<
		"z_corr" << m_offset_gyro.z();
	}

	if(!m_mean_Gaccel.isNull()){
		node = sxml["mean_g_accel"];
		node << "x_corr" << m_mean_Gaccel.x() <<
		"y_corr" << m_mean_Gaccel.y() <<
		"z_corr" << m_mean_Gaccel.z();
	}
}

void GyroData::draw_text(const Vector3d &v, QString text)
{
	QGLWidget *w = dynamic_cast< QGLWidget* >(parent());
	if(w){
		glColor3f(1, 1, 1);
		w->renderText(v.x(), v.y(), v.z(), text, QFont("Arial", 14));
	}
}

template < typename T >
inline T get_pt_sphere(double id, double jd, double R)
{
	double x = R * sin(id * 2 * M_PI) * sin(jd * M_PI);
	double y = R * cos(id * 2 * M_PI) * sin(jd * M_PI);
	double z = R * cos(jd * M_PI);
	return T(x, y, z);
}


void GyroData::init_sphere()
{
	if(m_sphere.isNull())
		return;

	const int count = 32;
	double R = m_sphere.mean_radius / m_divider_accel;

	m_vecs_sphere.clear();
	m_inds_sphere.clear();

	for(int i = 0; i < count; i++){
		double id = (double)i / (count - 1);
		for(int j = 0; j < count; j++){
			double jd = (double)j / (count - 1);
			m_vecs_sphere.push_back(get_pt_sphere<Vector3f>(id, jd, R));
		}
	}

	for(int i = 0; i < count - 1; i++){
		for(int j = 0; j < count - 1; j++){
			int A = i * count + j;
			int B = (i + 1) * count + j;
			int C = i * count + j + 1;
			m_inds_sphere.push_back(Vector3i(A, B, C));
		}
	}

	glVertexPointer(3, GL_FLOAT, sizeof(Vector3f), m_vecs_sphere.data()->data);
}

void GyroData::draw_sphere()
{
	if(!m_inds_sphere.size())
		return;

	glPushMatrix();

	Vector3d cp(m_sphere.cp);
	cp *= 1.0 / m_divider_accel;

	glTranslated(cp.x(), cp.y(), cp.z());

	glColor3f(1, 1, 1);

	int cnt = m_inds_sphere.size() * 3;
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawElements(GL_LINE_STRIP, cnt, GL_UNSIGNED_INT, m_inds_sphere.data());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}
