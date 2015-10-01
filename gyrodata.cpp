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

///////////////////////////////
Q_DECLARE_METATYPE(Vertex3i)

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

static inline Vertex3d _V(const Vertex3i& v)
{
	return Vertex3d(v);
}

void draw_line(const QVector3D& v1, const QVector3D& v2, const QColor& col = Qt::white)
{
	glColor3ub(col.red(), col.green(), col.blue());

	glBegin(GL_LINES);
	glVertex3d(v1.x(), v1.y(), v1.z());
	glVertex3d(v2.x(), v2.y(), v2.z());
	glEnd();
}

void draw_line(const Vertex3d& v1, const Vertex3d& v2, const QColor& col = Qt::white)
{
	glColor3ub(col.red(), col.green(), col.blue());

	glBegin(GL_LINES);
	glVertex3dv(v1.data);
	glVertex3dv(v2.data);
	glEnd();
}

inline Vertex3i rshift(const Vertex3i& val, int shift)
{
	Vertex3i res;
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
{
	setType(GYRODATA);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_timeout()));
	m_timer.start(300);

	connect(&m_timer_playing, SIGNAL(timeout()), this, SLOT(on_timeout_playing()));
	m_timer_playing.start(100);

	connect(&m_timer_calibrate, SIGNAL(timeout()), this, SLOT(on_timeout_calibrate()));
	m_timer_calibrate.setInterval(300);

	m_time_waiting_telemetry.start();

	m_fileName = "../data/data.csv";

	load_from_xml();
	load_calibrate();
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

		st.accel = (Vertex3i(a1, a2, a3));
		st.gyro =(Vertex3i(g1, g2, g3));
		st.temp = t;
		st.afs_sel = afs;
		st.fs_sel = fs;
		st.freq = f;
		st.tick = tm;

		m_downloaded_telemetries.push_back(st);

		ind++;
	}

//	normalize_vector(m_accel_data);
//	normalize_vector(m_gyro_data);

	file.close();
}

void GyroData::recalc_accel(double threshold, double threshold_deriv)
{
	Q_UNUSED(threshold);
	Q_UNUSED(threshold_deriv);
//	m_center_accel = get_center(m_accel_data, mean_radius);

//	bool loop = true;
//	double sub_prev = 1;
//	do{
//		QVector< QVector3D > out;
//		QVector< int > errors;
//		bool res = search_outliers(m_center_accel, mean_radius, threshold, m_accel_data, out, errors);

//		double new_mean = 0;
//		QVector3D newc = get_center(out, new_mean);

//		double sub = qAbs(new_mean - mean_radius);
//		double s1 = qAbs(sub_prev - sub);
//		loop = s1 > threshold_deriv;
//		sub_prev = sub;

//		m_center_accel = newc;
//		m_accel_data = out;
//		mean_radius = new_mean;
	//	}while(loop);
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
	m_offset_gyro = Vertex3d();
	m_count_gyro_offset_data = 0;
	m_rotate_pos = Vertex3d();
	m_translate_pos = Vertex3d();
	m_translate_speed = Vertex3f();
	m_mean_Gaccel = Vertex3d();
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

		m_rotate_pos = Vertex3d();
		m_translate_pos = Vertex3d();
		m_is_calculated = true;
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
	m_rotate_pos = Vertex3d();
	m_translate_pos = Vertex3d();
	m_translate_speed = Vertex3d();
	m_tmp_axes = Vertex3f(1, 0, 0);
	m_tmp_angle = 0;
	m_mean_accel = Vertex3d();
	m_past_tick = 0;
	m_first_tick = 0;
}

bool GyroData::calibrate()
{
	if(m_calibrate.is_progress())
		return false;

	m_calibrate.set_parameters(&m_downloaded_telemetries);

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
		init_sphere();

		save_calibrate();
	}
}

inline Vertex3d QV3V3(const QVector3D& v)
{
	return Vertex3d(v.x(), v.y(), v.z());
}

inline QVector3D V3QV3(const Vertex3d& v)
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


void GyroData::init()
{
	openFile(m_fileName);

	init_sphere();
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
			Vertex3d tmp(_V(m_downloaded_telemetries[i].accel) * div_accel);
			glVertex3dv(tmp.data);
		}
		glEnd();

		glColor3f(1, 0, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i< count; i++) {
			Vertex3d tmp(_V(m_downloaded_telemetries[i].gyro) * div_gyro);
			glVertex3dv(tmp.data);
		}
		glEnd();
	}

	glLineWidth(4);


	if(m_is_calculated){
		draw_line(Vertex3d(), m_translate_pos, Qt::magenta);
		draw_line(Vertex3d(), m_tmp_axes, Qt::yellow);
	}

	glPushMatrix();

	//glRotated(-m_tmp_angle, m_tmp_axes.x(), m_tmp_axes.y(), m_tmp_axes.z());

	glRotatef(m_rotate_pos.x(), 1, 0, 0);
	glRotatef(m_rotate_pos.y(), 0, 1, 0);
	glRotatef(m_rotate_pos.z(), 0, 0, 1);

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

	glLineWidth(1);


	glPointSize(4);

	if(m_telemetries.size()){

		Vertex3d tmp(_V(m_telemetries[0].gyro) * div_gyro);

		glColor3f(1, 0.5, 0);
		glBegin(GL_POINTS);
			glVertex3dv(tmp.data);
		glEnd();

		glLineWidth(3);
		draw_line(Vertex3d(), tmp, QColor(255, 128, 0));

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

		glColor3f(0.5, 1, 0);
		glBegin(GL_POINTS);
			tmp = _V(m_telemetries[0].accel) * div_accel;
			glVertex3dv(tmp.data);
		glEnd();

		glPushMatrix();

//		glRotatef(-m_rotate_pos.x(), 1, 0, 0);
//		glRotatef(-m_rotate_pos.y(), 0, 1, 0);
//		glRotatef(-m_rotate_pos.z(), 0, 0, 1);

		glLineWidth(3);
		draw_line(Vertex3d(), m_mean_accel * div_accel, QColor(128, 255, 128));
		draw_text(m_mean_accel * div_accel, "mean_accel");
		draw_line(Vertex3d(), m_tmp_accel * div_accel, QColor(255, 128, 0));
		draw_text(m_tmp_accel * div_accel, "tmp_accel. "
				  + QString::number(m_mean_Gaccel.length() - m_telemetries[0].accel.length()));

		draw_line(Vertex3d(), tmp, QColor(128, 255, 0));
		draw_text(tmp, "accel");

		glLineWidth(1);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < m_telemetries.size(); i++){
			StructTelemetry& st = m_telemetries[i];
			float dd = (float)(m_telemetries.size() - i) / m_telemetries.size();
			glColor3f(0.5 * dd, 1 * dd, 0);

			tmp = _V(st.accel) * div_accel;

			glVertex3dv(tmp.data);
		}
		glEnd();

		glPopMatrix();
	}

	if(m_is_draw_mean_sphere){
		draw_sphere();
	}
//	draw_line(p1, cp1m);
//	draw_line(p2, cp1m);
//	draw_line(p3, cp1m);
}

void GyroData::tick()
{
}

QVector3D GyroData::position() const
{
	return QVector3D();
}

void GyroData::calc_offsets(Vertex3i &gyro, Vertex3i &accel)
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
	m_offset_gyro = Vertex3d();
	m_rotate_pos = Vertex3d();
	m_mean_Gaccel = Vertex3d();
	m_tmp_axes = Vertex3f(1, 0, 0);
	m_tmp_angle = 0;
	m_mean_accel = Vertex3d();
	m_past_tick = 0;
	m_first_tick = 0;
	m_index = 0;
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

void GyroData::tryParseData(const QByteArray &data)
{
	if(!m_tick_telemetry.isValid() || m_tick_telemetry.hasExpired(max_delay_for_data)){
		m_tick_telemetry.restart();
	}

	StructTelemetry st;

	QDataStream stream(data);
	st.read_from(stream);

	qint64 tick_delta = m_tick_telemetry.nsecsElapsed();
	double part_of_time = tick_delta / 1e+9;
	m_part_of_time = part_of_time;

	m_tick_telemetry.restart();

	analyze_telemetry(st);

	WriteLog::instance()->add_data("gyro", st);

	m_time_waiting_telemetry.restart();

	m_index++;

	while(m_telemetries.size() >= max_count_telemetry){
		m_telemetries.pop_back();
	}
}

const int min_threshold_accel = 200;

void GyroData::analyze_telemetry(StructTelemetry &st)
{
	emit get_data("gyro", st.gyro);
	emit get_data("accel", st.accel);

	m_kalman[0].set_zk(st.accel.x());
	m_kalman[1].set_zk(st.accel.y());
	m_kalman[2].set_zk(st.accel.z());

	m_kalman[3].set_zk(st.gyro.x());
	m_kalman[4].set_zk(st.gyro.y());
	m_kalman[5].set_zk(st.gyro.z());

	Vertex3d kav(Vertex3d(m_kalman[0].xk, m_kalman[1].xk, m_kalman[2].xk));
	Vertex3d kgv(Vertex3d(m_kalman[3].xk, m_kalman[4].xk, m_kalman[5].xk));

	st.accel = kav;
	st.gyro = kgv;

	emit get_data("kalman_accel", kav);
	emit get_data("kalman_gyro", kgv);

	calc_offsets(st.gyro, st.accel);

	m_mean_accel = m_mean_accel * 0.9 + Vertex3d(st.accel) * 0.1;

	if(!m_is_calc_offset_gyro && m_is_calculated){
		m_rotate_pos -= (st.angular_speed(m_offset_gyro) * m_part_of_time);

		if(m_past_accel.isNull())
			m_past_accel = st.accel;

//		double lenG = m_mean_Gaccel.length();
//		double lenA = st.accel.length();
//		if(lenA - lenG < min_threshold_accel && lenA > 0.5 * lenG){
//			m_tmp_accel = m_tmp_accel * 0.9 + Vertex3d(st.accel) * 0.1;
//		}

//		Vertex3d staccel(st.accel);
//		Vertex3d offset = staccel - Vertex3d(m_past_accel);
//		m_past_accel = st.accel;

//		int offset_len = offset.length();
//		//offset_len = m_len_Gaccel - offset_len;
//		if(qAbs(offset_len) < min_threshold_accel){
//			offset_len = 0;
//		}
////		offset.normalize();
////		offset *= (double)offset_len;
//		m_translate_speed += offset;
//		m_translate_pos += (m_translate_speed * (1.0/m_divider_accel));

		QQuaternion qX, qY, qZ, qres;
		qX = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), m_rotate_pos.x());
		qY = QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), m_rotate_pos.y());
		qZ = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), m_rotate_pos.z());
		qres = qX * qY * qZ;
		Vertex3d norm(0, 0, -1), accel(m_mean_accel);

		norm = QV3V3(qres.rotatedVector(V3QV3(norm)));

		accel.normalize();
		norm.normalize();
		double angle = Vertex3d::dot(norm, accel);
		angle = acos(angle) * 180.0 / M_PI;
		if(fabs(angle) > 1e-7){
			Vertex3d axes = Vertex3d::cross(accel, norm).normalized();
			m_tmp_axes = axes;
		}
		m_tmp_angle = angle;
	}

	m_telemetries.push_front(st);

//	if(m_telemetries.size() > 10)
//		m_telemetries.pop_back();
}

void GyroData::load_from_xml()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir + xml_config;

	SimpleXML sxml(config_file);

	if(!sxml.load())
		return;

	set_end_pos_downloaded_data(sxml.get_xml_double("end_position") * 100.0);
	m_divider_accel = sxml.get_xml_double("divider_accel");
	m_divider_gyro = sxml.get_xml_double("divider_gyro");
	m_fileName = sxml.get_xml_string("filename");
	m_showing_downloaded_data = sxml.get_xml_int("showing_downloaded_data");
	m_addr = QHostAddress(sxml.get_xml_string("ip"));
	ushort port = sxml.get_xml_int("port");

	if(m_addr.isNull()){
		m_addr = QHostAddress("192.168.0.200");
	}
	if(port)
		m_port = port;

	double freq = sxml.get_xml_double("freq_playing");
	if(freq){
		set_freq_playing(freq);
	}

	m_is_draw_mean_sphere = sxml.get_xml_int("draw_mean_sphere");
}

void GyroData::save_to_xml()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir;

	QDir dir(QDir::homePath());

	//if(!dir.exists(config_file))
	dir.mkdir(config_file);

	config_file += xml_config;

	SimpleXML sxml(config_file, true);

	sxml.set_dom_value_num("end_position", m_percent_downloaded_data);
	sxml.set_dom_value_num("divider_accel", m_divider_accel);
	sxml.set_dom_value_num("divider_gyro", m_divider_gyro);
	sxml.set_dom_value_s("filename", m_fileName);
	sxml.set_dom_value_num("showing_downloaded_data", m_showing_downloaded_data);
	sxml.set_dom_value_s("ip", m_addr.toString());
	sxml.set_dom_value_num("port", m_port);

	double freq = freq_playing();
	sxml.set_dom_value_num("freq_playing", freq);
	sxml.set_dom_value_num("draw_mean_sphere", m_is_draw_mean_sphere);

	sxml.save();
}

void GyroData::load_calibrate()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir + xml_calibrate;

	SimpleXML sxml(config_file);

	if(!sxml.load())
		return;

	Vertex3d v;
	v.setX(sxml.get_xml_double("x"));
	v.setY(sxml.get_xml_double("y"));
	v.setZ(sxml.get_xml_double("z"));
	m_sphere.cp = v;
	m_sphere.mean_radius = sxml.get_xml_double("mean_radius");
	m_sphere.deviation = sxml.get_xml_double("deviation");
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

	SimpleXML sxml(config_file, true);

	sxml.set_dom_value_num("x", m_sphere.cp.x());
	sxml.set_dom_value_num("y", m_sphere.cp.y());
	sxml.set_dom_value_num("z", m_sphere.cp.z());
	sxml.set_dom_value_num("mean_radius", m_sphere.mean_radius);
	sxml.set_dom_value_num("deviation", m_sphere.deviation);

	sxml.save();
}

void GyroData::draw_text(const Vertex3d &v, QString text)
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
	double x = R * sin(id * 2 * M_PI) * cos(jd * 2 * M_PI);
	double y = R * cos(id * 2 * M_PI) * cos(jd * 2 * M_PI);
	double z = R * sin(jd * 2 * M_PI);
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
		double id = (double)i / count;
		for(int j = 0; j < count; j++){
			double jd = (double)j / count;
			m_vecs_sphere.push_back(get_pt_sphere<Vertex3f>(id, jd, R));
		}
	}

	for(int i = 0; i < count - 1; i++){
		for(int j = 0; j < count - 1; j++){
			int A = i * count + j;
			int B = (i + 1) * count + j;
			int C = i * count + j + 1;
			m_inds_sphere.push_back(Vertex3i(A, B, C));
		}
	}

	glVertexPointer(3, GL_FLOAT, sizeof(Vertex3f), m_vecs_sphere.data()->data);
}

void GyroData::draw_sphere()
{
	if(!m_inds_sphere.size())
		return;

	glPushMatrix();

	Vertex3d cp(m_sphere.cp);
	cp *= 1.0 / m_divider_accel;

	glTranslated(cp.x(), cp.y(), cp.z());

	glColor3f(1, 1, 1);

	int cnt = m_inds_sphere.size() * 3;
	glEnableClientState(GL_VERTEX_ARRAY);
	//glDrawRangeElements(GL_LINE_LOOP, 0, cnt, cnt, GL_INT, m_inds_sphere.data());
//	glDrawArrays(GL_LINE_STRIP, 0, 40);
	glDrawElements(GL_LINE_STRIP, cnt, GL_UNSIGNED_INT, m_inds_sphere.data());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}
