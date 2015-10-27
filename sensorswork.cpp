#include "sensorswork.h"
#include <QThreadPool>

#include "simple_xml.hpp"

using namespace vector3_;
using namespace matrix;
using namespace sc;
using namespace std;
using namespace sc;
using namespace quaternions;

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
{
	connect(this, SIGNAL(bind_address()), this, SLOT(_on_bind_address()), Qt::QueuedConnection);
	connect(this, SIGNAL(send_to_socket(QByteArray)), this, SLOT(_on_send_to_socket(QByteArray)), Qt::QueuedConnection);

	load_calibrate();
}

SensorsWork::~SensorsWork()
{
	quit();
	wait();

	if(m_socket){
		m_socket->abort();
	}
	delete m_socket;
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

	m_calibrate.set_parameters(&data);

	m_typeOfCalibrate = Accelerometer;

	m_timer_calibrate.start();

	QThreadPool::globalInstance()->start(new CalibrateAccelerometerRunnable(&m_calibrate));

	return true;
}

bool SensorsWork::calibrate_compass(const QVector<Vector3d> &data)
{
	if(m_calibrate.is_progress() || !data.size())
		return false;

	m_calibrate.set_parameters(data);

	m_typeOfCalibrate = Compass;

	m_timer_calibrate.start();

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

SensorsWork::TypeOfCalibrate GyroData::typeOfCalibrate() const
{
	return m_typeOfCalibrate;
}

void SensorsWork::run()
{
	m_timer = new QTimer;
	m_timer_calibrate = new QTimer;

	connect(m_timer, SIGNAL(timeout()), this, SLOT(_on_timeout()));
	m_timer.start(300);

	connect(m_timer_calibrate, SIGNAL(timeout()), this, SLOT(_on_timeout_calibrate()));
	m_timer_calibrate->setInterval(300);

	m_time_waiting_telemetry->start();

	connect(&m_calibrate, SIGNAL(send_log(QString)), this, SLOT(on_calibrate_log(QString)));

	m_socket = new QUdpSocket;
	connect(m_socket, SIGNAL(readyRead()), this, SLOT(_on_readyRead()));
	m_socket->bind(m_receiver_port);

	exec();
}

void SensorsWork::reset_mean_sphere()
{
	m_sphere.reset();
}

void SensorsWork::_on_timeout()
{
	if(m_telemetries.size() > 3){
		m_telemetries.pop_back();
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

bool SensorsWork::is_exists_value(GyroData::POS pos) const
{
	return !m_pos_values[pos].isNull();
}

void SensorsWork::set_start_calc_pos(GyroData::POS pos)
{
	m_curcalc_pos = pos;
	m_pos_values[pos] = Vector3d();
	m_calcid = 0;
	m_is_calc_pos = true;
}

void SensorsWork::set__position()
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
	m_mean_Gaccel = Vector3d();
	m_rotate_quaternion = Quaternion();
	m_len_Gaccel = 0;
}

void SensorsWork::stop_calc_offset_gyro()
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
		calc_correction();
	}
}

void SensorsWork::calc_parameters()
{
	Vector3d v1(1, 0, 0), v2, v3(0, 1, 0);
	v2 = m_rotate_quaternion.rotatedVector(v1);
	double an = common_::rad2angle(atan2(v2.z(), v2.x()));
	set_text("tangaj", QString::number(an, 'f', 1));

	v2 = m_rotate_quaternion.rotatedVector(v3);
	an = common_::rad2angle(atan2(v2.z(), v2.y()));
	set_text("bank", QString::number(an, 'f', 1));
}

void SensorsWork::calc_correction()
{
	if(m_mean_Gaccel.isNull())
		return;
	Vector3d vnorm(0, 0, -1), vmg = m_mean_Gaccel.normalized(), v;
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
	Vector3d vga = m_rotate_quaternion.rotatedVector(Vector3d(0, 0, -1)).normalized();
	Vector3d va = m_mean_accel.normalized();
	Vector3d ax = Vector3d::cross(va, vga).normalized();
	double an = Vector3d::dot(vga, va);
	an = common_::rad2angle(acos(an));
	m_accel_quat = Quaternion::fromAxisAndAngle(ax, -an) * m_rotate_quaternion;
	m_accel_quat.normalize();

	if(!m_is_start_correction && fabs(an) > m_max_threshold_angle){
		m_is_start_correction = true;
	}

	if(m_is_start_correction && m_tmp_accel.length() < m_threshold_accel * m_sphere.mean_radius){
		m_rotate_quaternion = Quaternion::slerp(m_rotate_quaternion, m_accel_quat, m_multiply_correction);

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
		m_max_threshold_angle = node["min_threshold_angle"];

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

				m_sphereGl.generate_sphere(m_sphere.mean_radius, m_divider_accel);

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
		m_timer_calibrate.stop();
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

	//remove_lowbits(st.gyroscope.accel, 7);
//	remove_lowbits(st.gyroscope.gyro, 4);
//	qint64 tick_delta = m_tick_telemetry.nsecsElapsed();
//	double part_of_time = tick_delta / 1e+9;
//	m_part_of_time = part_of_time;

//	m_tick_telemetry.restart();
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

	st = analyze_telemetry(st);

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
	if(!common_::fIsNull(aspeed)){
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

StructTelemetry SensorsWork::analyze_telemetry(const StructTelemetry &st_in)
{
	StructTelemetry st(st_in);

	fill_data_for_calibration(st);

	/// a subtraction of the offset error of acceleration of the sensor
	st.gyroscope.accel -= m_sphere.cp;

	//Vector3d v = st.gyroscope.accel;
	//v = m_corr_matrix * v;
	//st.gyroscope.accel = v;

	simple_kalman_filter(st, st);

	calc_offsets(st.gyroscope.gyro, st.gyroscope.accel);

	Vector3d v = st.gyroscope.accel;

	if(m_mean_accel.isNull())
		m_mean_accel = v;
	m_mean_accel = m_mean_accel * 0.9 + v * 0.1;

	if(!m_is_calc_offset_gyro && m_is_calculated){
		//m_rotate_pos -= (st.gyroscope.angular_speed(m_offset_gyro) * m_part_of_time);
		Vector3d rotate_speed = (st.gyroscope.angular_speed(m_offset_gyro) * m_part_of_time);
		rotate_speed = rotate_speed.inv();

		Quaternion quat_speed = fromAnglesAxes(rotate_speed);
		m_rotate_quaternion *= quat_speed;

		Vector3d accel_mg = m_rotate_quaternion.conj().rotatedVector(m_mean_accel);

		if(!m_prev_accel.isNull()){
			m_tmp_accel = (accel_mg - m_prev_accel);
		}
		m_prev_accel = accel_mg;

		/////////////////////////////

		calccount(st);
		calc_parameters();

		/////////////////////////////

		correct_error_gyroscope();
	}

	m_telemetries.push_front(st);

	return st;
}

void SensorsWork::clear_data()
{
	m_count_gyro_offset_data = 0;
	m_rotate_pos = Vector3d();
	m_tmp_axis = Vector3f(1, 0, 0);
	m_tmp_angle = 0;
	m_mean_accel = Vector3d();
	m_past_tick = 0;
	m_first_tick = 0;
	m_translate_pos = Vector3d();
	m_translate_speed = Vector3d();
	m_rotate_quaternion = Quaternion();

	m_is_start_correction = false;
}

void SensorsWork::calc_offsets(const Vector3i &gyro, const Vector3i &accel)
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
