#include "sensorswork.h"
#include <QThreadPool>

#include <QQuaternion>
#include <QMatrix4x4>
#include <QApplication>
#include <QDir>
#include <QFile>

#include "simple_xml.hpp"

#include "global.h"
#include "writelog.h"

using namespace vector3_;
using namespace matrix;
using namespace sc;
using namespace std;
using namespace sc;
using namespace quaternions;

const QString xml_calibrate("calibrate.xml");

/////////////////////////////////////////////////////

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

matrix::Matrix3d fromQuaternionM3(const Quaternion &q_in)
{
	Quaternion q = q_in.normalized();
	double qi2 = q.x() * q.x();
	double qj2 = q.y() * q.y();
	double qk2 = q.z() * q.z();

	double qij = q.x() * q.y();
	double qik = q.x() * q.z();
	double qjk = q.y() * q.z();

	double qir = q.w * q.x();
	double qjr = q.w * q.y();
	double qkr = q.w * q.z();

	double data[] = {
		1.0 - 2.0 * (qj2 + qk2), 2.0 * (qij - qkr), 2.0 * (qik + qjr),
		2.0 * (qij + qkr), 1.0 - 2.0 * (qi2 + qk2), 2.0 * (qjk - qir),
		2.0 * (qik - qjr), 2.0 * (qjk + qir), 1.0 - 2.0 * (qi2 + qj2),
	};

	matrix::Matrix3d m(data);

	return m;
}

/////////////////////////////////////////////////////

SensorsWork::SensorsWork(QObject *parent)
	: QThread(parent)
	, m_socket(0)
	, m_is_calc_offset_gyro(false)
	, m_count_gyro_offset_data(0)
	, m_is_calculated(false)
	, m_tmp_angle(0)
	, m_port(7777)
	, m_addr(QHostAddress("192.168.0.200"))
	, m_part_of_time(5.0/1000.0)	/// default: (5/1000)
	, m_past_tick(0)
	, m_first_tick(0)
	, m_threshold_accel(0.07)
	, m_is_calc_pos(false)
	, m_calccount(100)
	, m_calcid(0)
	, m_curcalc_pos(POS_0)
	, m_is_start_correction(false)
	, m_receiver_port(7770)
	, m_typeOfCalibrate(NONE)
	, m_max_threshold_angle(3)
	, m_min_threshold_angle(1e-1)
	, m_multiply_correction(0.7)
	, m_index(0)
{
	connect(this, SIGNAL(bind_address()), this, SLOT(_on_bind_address()), Qt::QueuedConnection);
	connect(this, SIGNAL(send_to_socket(QByteArray)), this, SLOT(_on_send_to_socket(QByteArray)), Qt::QueuedConnection);
	connect(this, SIGNAL(start_calibration_watcher()), this, SLOT(_on_start_calibration_watcher()), Qt::QueuedConnection);

	load_calibrate();
}

SensorsWork::~SensorsWork()
{
	QThreadPool::globalInstance()->waitForDone();

	quit();
	wait();

	if(m_timer){
		delete m_timer;
	}
	if(m_timer_calibrate){
		delete m_timer_calibrate;
	}
}

int SensorsWork::count_gyro_offset_data() const
{
	return m_count_gyro_offset_data;
}

bool SensorsWork::is_available_telemetry() const
{
	return m_time_waiting_telemetry.elapsed() < max_delay_for_data;
}

QHostAddress SensorsWork::addr() const
{
	return m_addr;
}

ushort SensorsWork::port() const
{
	return m_receiver_port;
}

void SensorsWork::send_start()
{
	emit send_to_socket(QByteArray("START"));
}

void SensorsWork::send_stop()
{
	emit send_to_socket(QByteArray("STOP"));
}

void SensorsWork::set_address(const QHostAddress &address, ushort port)
{
	m_addr = address;
	m_receiver_port = port;

	emit bind_address();
}

bool SensorsWork::calibrate_accelerometer(const QVector<Vector3d> &data)
{
	if(m_calibrate.is_progress() || !data.size())
		return false;

	m_calibrate.set_parameters(data);

	m_typeOfCalibrate = Accelerometer;

	emit start_calibration_watcher();

	QThreadPool::globalInstance()->start(new CalibrateAccelerometerRunnable(&m_calibrate));

	return true;
}

bool SensorsWork::calibrate_compass(const QVector<Vector3d> &data)
{
	if(m_calibrate.is_progress() || !data.size())
		return false;

	m_calibrate.set_parameters(data);

	m_typeOfCalibrate = Compass;

	emit start_calibration_watcher();

	QThreadPool::globalInstance()->start(new CalibrateAccelerometerRunnable(&m_calibrate));

	return true;

}

void SensorsWork::reset_calibration_compass()
{
	m_sphere_compass.reset();
}

const CalibrateAccelerometer &SensorsWork::calibrate_thread() const
{
	return m_calibrate;
}

SensorsWork::TypeOfCalibrate SensorsWork::typeOfCalibrate() const
{
	return m_typeOfCalibrate;
}

void SensorsWork::run()
{
	m_timer = new QTimer;
	m_timer_calibrate = new QTimer;

	connect(m_timer, SIGNAL(timeout()), this, SLOT(_on_timeout()));
	m_timer->start(300);

	connect(m_timer_calibrate, SIGNAL(timeout()), this, SLOT(_on_timeout_calibrate()));
	m_timer_calibrate->setInterval(300);

	m_time_waiting_telemetry.start();

	m_socket = new QUdpSocket;
	connect(m_socket, SIGNAL(readyRead()), this, SLOT(_on_readyRead()));
	m_socket->bind(m_receiver_port);

	exec();

	if(m_socket){
		m_socket->abort();
		delete m_socket;
	}
}

void SensorsWork::reset_mean_sphere()
{
	m_sphere.reset();
}

void SensorsWork::_on_timeout()
{
	if(telemetries.size() > 3){
		telemetries.pop_back();
	}
}

void SensorsWork::_on_bind_address()
{
	m_socket->bind(m_receiver_port);
}

void SensorsWork::_on_send_to_socket(const QByteArray &data)
{
	m_socket->writeDatagram(data, m_addr, m_port);
}

void SensorsWork::_on_start_calibration_watcher()
{
	m_timer_calibrate->start();
}

bool SensorsWork::is_exists_value(SensorsWork::POS pos) const
{
	return !m_pos_values[pos].isNull();
}

void SensorsWork::set_start_calc_pos(SensorsWork::POS pos)
{
	m_curcalc_pos = pos;
	m_pos_values[pos] = Vector3d();
	m_calcid = 0;
	m_is_calc_pos = true;
}

void SensorsWork::set_position()
{
	m_rotate_pos = Vector3d();
	m_translate_pos = Vector3d();
	m_translate_speed = Vector3d();
	m_tmp_axis = Vector3f(1, 0, 0);
	m_tmp_angle = 0;
	mean_accel = Vector3d();
	m_past_tick = 0;
	m_first_tick = 0;
	rotate_quaternion = Quaternion();
}

void SensorsWork::start_calc_offset_gyro()
{
	m_is_calc_offset_gyro = true;
	m_is_calculated = false;
	m_offset_gyro = Vector3d();
	m_count_gyro_offset_data = 0;
	m_rotate_pos = Vector3d();
	m_translate_pos = Vector3d();
	m_translate_speed = Vector3f();
	meanGaccel = Vector3d();
	rotate_quaternion = Quaternion();
	m_len_Gaccel = 0;
}

void SensorsWork::stop_calc_offset_gyro()
{
	m_is_calc_offset_gyro = false;
	if(m_count_gyro_offset_data){
		m_offset_gyro *= 1.0/m_count_gyro_offset_data;
		meanGaccel *= 1.0/m_count_gyro_offset_data;

		m_len_Gaccel = meanGaccel.length();

		tmp_accel = meanGaccel;

		m_rotate_pos = Vector3d();
		rotate_quaternion = Quaternion();
		m_translate_pos = Vector3d();
		m_is_calculated = true;

		Vector3d v = meanGaccel;
		emit add_to_log("offset values.  gyroscope: " + QString::number(m_offset_gyro.x(), 'f', 3) + ", " +
						QString::number(m_offset_gyro.y(), 'f', 3) + ", " +
						QString::number(m_offset_gyro.z(), 'f', 3) + "; accelerometer: " +
						QString::number(v.x(), 'f', 3) + ", " +
						QString::number(v.y(), 'f', 3) + ", " +
						QString::number(v.z(), 'f', 3));
		calc_correction();
	}
}

void SensorsWork::calc_correction()
{
	if(meanGaccel.isNull())
		return;
	Vector3d vnorm(0, 0, -1), vmg = meanGaccel.normalized(), v;
	double angle = Vector3d::dot(vnorm, vmg);
	angle = common_::rad2angle(acos(angle));
	if(!common_::fIsNull(angle)){
		v = Vector3d::cross(vnorm, vmg).normalized();
		m_correct_quaternion = Quaternion::fromAxisAndAngle(v, -angle);
		m_corr_matrix = fromQuaternionM3(m_correct_quaternion);
		//m_corr_matrix.ident();
		//m_mean_Gaccel = m_corr_matrix * m_mean_Gaccel;
	}
}

void SensorsWork::correct_error_gyroscope()
{
	Vector3d vga = rotate_quaternion.rotatedVector(Vector3d(0, 0, -1)).normalized();
	Vector3d va = mean_accel.normalized();
	Vector3d ax = Vector3d::cross(va, vga).normalized();
	double an = Vector3d::dot(vga, va);
	an = common_::rad2angle(acos(an));
	accel_quat = Quaternion::fromAxisAndAngle(ax, -an) * rotate_quaternion;
	accel_quat.normalize();

	if(!m_is_start_correction && fabs(an) > m_max_threshold_angle){
		m_is_start_correction = true;
	}

	if(m_is_start_correction && tmp_accel.length() < m_threshold_accel * m_sphere.mean_radius){
		rotate_quaternion = Quaternion::slerp(rotate_quaternion, accel_quat, m_multiply_correction);

		//qDebug() << an;
		if(fabs(an) < m_min_threshold_angle){
			m_is_start_correction = false;
		}
	}
}

void SensorsWork::calccount(const StructTelemetry &st)
{
	if(m_is_calc_pos){
		m_pos_values[m_curcalc_pos] += Vector3d(st.gyroscope.accel);
		m_calcid++;
		if(m_calcid >= m_calccount){
			m_is_calc_pos = 0;
			m_pos_values[m_curcalc_pos] *= 1. / m_calcid;
			m_is_calc_pos = false;
			emit add_to_log(QString::number(m_curcalc_pos) + ": " + m_pos_values[m_curcalc_pos] +
							", len: " + QString::number(m_pos_values[m_curcalc_pos].length(), 'f', 1));
		}
	}
}

void SensorsWork::load_calibrate()
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
	if(!node["threshold_delta_accel"].empty())
		m_threshold_accel = node["threshold_delta_accel"];
	if(!node["max_threshold_angle"].empty())
		m_max_threshold_angle = node["max_threshold_angle"];
	if(!node["min_threshold_angle"].empty())
		m_min_threshold_angle = node["min_threshold_angle"];
	if(!node["multiply_correction"].empty())
		m_multiply_correction = node["multiply_correction"];

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
		meanGaccel = v;
	}else
		fl = false;

	if(fl){
		m_is_calculated = true;
		m_is_calc_offset_gyro = false;
	}

	if(!sxml["compass"].empty()){
		node = sxml["compass"];
		m_sphere_compass.cp.setX((double)node["x_corr"]);
		m_sphere_compass.cp.setY(node["y_corr"]);
		m_sphere_compass.cp.setZ(node["z_corr"]);
		m_sphere_compass.mean_radius = node["mean_radius"];
		m_sphere_compass.deviation = node["deviation"];
	}

	calc_correction();
}

void SensorsWork::save_calibrate()
{
	if(m_sphere.empty())
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
	node << "threshold_delta_accel" << m_threshold_accel;
	node << "max_threshold_angle" << m_max_threshold_angle;
	node << "min_threshold_angle" << m_min_threshold_angle;
	node << "multiply_correction" << m_multiply_correction;

	if(!m_offset_gyro.isNull()){
		node = sxml["gyroscope"];
		node << "x_corr" << m_offset_gyro.x() <<
		"y_corr" << m_offset_gyro.y() <<
		"z_corr" << m_offset_gyro.z();
	}

	if(!meanGaccel.isNull()){
		node = sxml["mean_g_accel"];
		node << "x_corr" << meanGaccel.x() <<
		"y_corr" << meanGaccel.y() <<
		"z_corr" << meanGaccel.z();
	}

	if(!m_sphere_compass.empty()){
		SimpleXMLNode node = sxml["compass"];
		node << "x_corr" <<  m_sphere_compass.cp.x() << "y_corr" <<  m_sphere_compass.cp.y() << "z_corr" << m_sphere_compass.cp.z();
		node << "mean_radius" << m_sphere_compass.mean_radius;
		node << "deviation" << m_sphere_compass.deviation;
	}
}

void SensorsWork::_on_timeout_calibrate()
{
	if(m_calibrate.is_done()){

		switch (m_typeOfCalibrate) {
			case Accelerometer:
				m_sphere = m_calibrate.result();
				emit add_to_log("evaluate accelerometer. x=" + QString::number(m_sphere.cp.x(), 'f', 3) +
						   ", y=" + QString::number(m_sphere.cp.y(), 'f', 3) +
						   ", z=" + QString::number(m_sphere.cp.z(), 'f', 3) +
						   "; R=" + QString::number(m_sphere.mean_radius, 'f', 3) +
						   "; dev=" + QString::number(m_sphere.deviation, 'f', 3));

				break;
			case Compass:
				m_sphere_compass = m_calibrate.result();
				emit add_to_log("evaluate compass. x=" + QString::number(m_sphere_compass.cp.x(), 'f', 3) +
						   ", y=" + QString::number(m_sphere_compass.cp.y(), 'f', 3) +
						   ", z=" + QString::number(m_sphere_compass.cp.z(), 'f', 3) +
						   "; R=" + QString::number(m_sphere_compass.mean_radius, 'f', 3) +
						   "; dev=" + QString::number(m_sphere_compass.deviation, 'f', 3));
			default:
				break;
		}

		save_calibrate();
		m_timer_calibrate->stop();

		emit stop_calibration();
	}
}

void remove_lowbits(Vector3i& v, int bits)
{
	FOREACH(i, Vector3i::count, v.data[i] = (v.data[i] >> bits) << bits);
}

void SensorsWork::_on_readyRead()
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

void SensorsWork::tryParseData(const QByteArray &data)
{
	if(!m_tick_telemetry.isValid() || m_tick_telemetry.hasExpired(max_delay_for_data)){
		m_tick_telemetry.restart();
	}

	StructTelemetry st;

	QDataStream stream(data);
	st.read_from(stream);

	st = analyze_telemetry(st);

	WriteLog::instance()->add_data("gyro", st);

	m_time_waiting_telemetry.restart();

	m_index++;

	while(telemetries.size() >= max_count_telemetry){
		telemetries.pop_back();
	}
}

const int min_threshold_accel = 200;

static inline Quaternion fromAnglesAxes(const Vector3d& angles)
{
	Quaternion qX, qY, qZ, qres;
	double aspeed = angles.length();
	if(!common_::fIsNull(aspeed)){
		Vector3d axis = angles.normalized();
		qres = Quaternion::fromAxisAndAngle(axis, -aspeed);
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

StructTelemetry SensorsWork::analyze_telemetry(const StructTelemetry &st_in)
{
	StructTelemetry st(st_in);

	if(st.gyroscope.tick && m_first_tick){
		long long tick = st.gyroscope.tick - m_first_tick;
		//qDebug() << m_index << tick - m_past_tick << st.gyroscope.tick << m_first_tick + m_past_tick;
		m_part_of_time = (double)(tick - m_past_tick) / 1e+3;
		m_past_tick = tick;
	}else{
		if(st.gyroscope.tick){
			m_first_tick = st.gyroscope.tick;
			m_past_tick = st.gyroscope.tick - m_first_tick;
		}
	}

	emit fill_data_for_calibration(st);

	/// a subtraction of the offset error of acceleration of the sensor
	st.gyroscope.accel -= m_sphere.cp;

	//Vector3d v = st.gyroscope.accel;
	//v = m_corr_matrix * v;
	//st.gyroscope.accel = v;

	simple_kalman_filter(st, st);

	calc_offsets(st.gyroscope.gyro, st.gyroscope.accel);

	Vector3d v = st.gyroscope.accel;

	if(mean_accel.isNull())
		mean_accel = v;
	mean_accel = mean_accel * 0.9 + v * 0.1;

	if(!m_is_calc_offset_gyro && m_is_calculated){
		//m_rotate_pos -= (st.gyroscope.angular_speed(m_offset_gyro) * m_part_of_time);
		Vector3d rotate_speed = (st.gyroscope.angular_speed(m_offset_gyro) * m_part_of_time);
		rotate_speed = rotate_speed.inv();

		Quaternion quat_speed = fromAnglesAxes(rotate_speed);
		rotate_quaternion *= quat_speed;

		Vector3d accel_mg = rotate_quaternion.conj().rotatedVector(mean_accel);

		if(!m_prev_accel.isNull()){
			tmp_accel = (accel_mg - m_prev_accel);
		}
		m_prev_accel = accel_mg;

		/////////////////////////////

		calccount(st);

		/////////////////////////////

		//correct_error_gyroscope();
	}

	telemetries.push_front(st);

	while(telemetries.size() >= max_count_telemetry){
		telemetries.pop_back();
	}

	return st;
}

void SensorsWork::clear_data()
{
	m_count_gyro_offset_data = 0;
	m_rotate_pos = Vector3d();
	m_tmp_axis = Vector3f(1, 0, 0);
	m_tmp_angle = 0;
	mean_accel = Vector3d();
	m_past_tick = 0;
	m_first_tick = 0;
	m_translate_pos = Vector3d();
	m_translate_speed = Vector3d();
	rotate_quaternion = Quaternion();

	m_is_start_correction = false;
}

void SensorsWork::calc_offsets(const Vector3i &gyro, const Vector3i &accel)
{
	if(m_is_calc_offset_gyro){
		m_offset_gyro += gyro;
		meanGaccel += accel;

		m_count_gyro_offset_data++;
	}else{
//		gyro -= m_offset_gyro;
//		accel -= m_offset_accel;
	}
}

void SensorsWork::simple_kalman_filter(const StructTelemetry &st, StructTelemetry &st_out)
{
	emit get_data("gyro", st.gyroscope.gyro);
	emit get_data("accel", st.gyroscope.accel);
	emit get_data("compass", st.compass.data);

	Vector3d kav = m_kalman[0].set_zk(st.gyroscope.accel);
	Vector3d kgv = m_kalman[1].set_zk(st.gyroscope.gyro);
	Vector3d kcv = m_kalman[2].set_zk(st.compass.data);

	st_out.gyroscope.accel = kav;
	st_out.gyroscope.gyro = kgv;
	st_out.compass.data = kcv;

	emit get_data("kalman_accel", kav);
	emit get_data("kalman_gyro", kgv);
	emit get_data("kalman_compass", kcv);
}
