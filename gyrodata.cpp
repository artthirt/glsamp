#include "gyrodata.h"

#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QByteArray>
#include <QStringList>
#include <QColor>
#include <QDebug>
#include <QApplication>

#include <QUdpSocket>
#include <QDataStream>

#include "writelog.h"

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
#include <Windows.h>
#else
#include <chrono>
#endif

#include <GL/gl.h>

#include <global.h>
#include <simple_xml.hpp>

///////////////////////////////
Q_DECLARE_METATYPE(Vertex3i)

const QString xml_config("gyro.xml");

///////////////////////////////

void draw_line(const QVector3D& v1, const QVector3D& v2, const QColor& col = Qt::white)
{
	glColor3ub(col.red(), col.green(), col.blue());

	glBegin(GL_LINES);
	glVertex3d(v1.x(), v1.y(), v1.z());
	glVertex3d(v2.x(), v2.y(), v2.z());
	glEnd();
}

void draw_line(const Vertex3f& v1, const Vertex3f& v2, const QColor& col = Qt::white)
{
	glColor3ub(col.red(), col.green(), col.blue());

	glBegin(GL_LINES);
	glVertex3fv(v1.data);
	glVertex3fv(v2.data);
	glEnd();
}

inline Vertex3i rshift(const Vertex3i& val, int shift)
{
	Vertex3i res;
	FOREACH(i, 3, res.data[i] = val.data[i] >> shift);
	return res;
}

///////////////////////////////
/// \brief GyroData::GyroData
/// \param parent
///
///
GyroData::GyroData(QObject *parent) :
	VirtGLObject(parent)
  , m_socket(0)
  , m_divider_accel(100)
  , m_divider_gyro(10)
  , m_shift_accel(5)
  , m_shift_gyro(3)
  , m_percent_downloaded_data(1)
  , m_showing_downloaded_data(true)
  , m_is_play(false)
  , m_current_playing_pos(0)
  , m_is_calc_offset_gyro(false)
  , m_count_gyro_offset_data(0)
{
	setType(GYRODATA);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_timeout()));
	m_timer.start(300);

	connect(&m_timer_playing, SIGNAL(timeout()), this, SLOT(on_timeout_playing()));
	m_timer_playing.start(100);

	m_time_waiting_telemetry.start();

	m_fileName = "../data/data.csv";

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

		if(sl.size() == 7){
			a1	= sl[0].toDouble();
			a2	= sl[1].toDouble();
			a3	= sl[2].toDouble();
			t	= sl[3].toDouble();
			g1	= sl[4].toDouble();
			g2	= sl[5].toDouble();
			g3	= sl[6].toDouble();
		}
		if(sl.size() >= 10){
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

		m_downloaded_telemetries.push_back(st);

		ind++;
	}

//	normalize_vector(m_accel_data);
//	normalize_vector(m_gyro_data);

	file.close();
}

inline QVector3D min_v(const QVector3D& v1, const QVector3D& v2)
{
	return QVector3D(
				qMin(v1.x(), v2.x()),
				qMin(v1.y(), v2.y()),
				qMin(v1.z(), v2.z())
				);
}

inline QVector3D max_v(const QVector3D& v1, const QVector3D& v2)
{
	return QVector3D(
				qMax(v1.x(), v2.x()),
				qMax(v1.y(), v2.y()),
				qMax(v1.z(), v2.z())
				);
}

void search_min_pt(const QVector< QVector3D >& data, QVector3D& min, QVector3D& max)
{
	 QVector3D res_min = data[0];
	 QVector3D res_max = res_min;

	 foreach (QVector3D it, data) {
		res_min = min_v(res_min, it);
		res_max = max_v(res_max, it);
	 }
	 min = res_min;
	 max = res_max;
}

bool search_outliers(const QVector3D& center, double mean_radius, double threshold,
					 QVector< QVector3D >& data, QVector< QVector3D >& out, QVector< int >& errors)
{
	if(!data.size() || qFuzzyIsNull(mean_radius))
		return false;

	out.clear();
	errors.resize(data.size());
	int i = 0;
	foreach (QVector3D it, data) {
		QVector3D vec = it - center;
		double len = vec.length();
		//double a = len / mean_radius;

		if(qAbs(len - mean_radius) > threshold){
			errors[i] = 1;
		}else{
			errors[i] = 0;
			out.push_back(it);
		}
		i++;
	}
	return true;
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

int GyroData::shift_gyro() const
{
	return m_shift_gyro;
}

void GyroData::set_shift_gyro(int val)
{
	m_shift_gyro = val;
}

int GyroData::shift_accel() const
{
	return m_shift_accel;
}

void GyroData::set_shift_accel(int val)
{
	m_shift_accel = val;
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
		m_telemtries.clear();
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
	m_offset_gyro = Vertex3f();
	m_count_gyro_offset_data = 0;
	m_rotate_pos = Vertex3f();
}

void GyroData::stop_calc_offset_gyro()
{
	m_is_calc_offset_gyro = false;
	if(m_count_gyro_offset_data){
		m_offset_gyro *= 1.0/m_count_gyro_offset_data;
		m_rotate_pos = Vertex3f();
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

void GyroData::set_zero_pos()
{
	m_rotate_pos = Vertex3f();
}

void GyroData::on_timeout()
{
	if(m_telemtries.size() > 3){
		m_telemtries.pop_back();
	}
}

void GyroData::on_timeout_playing()
{
	if(m_is_play && (m_downloaded_telemetries.size())){

		StructTelemetry st = m_downloaded_telemetries[m_current_playing_pos];;

		emit get_data("gyro", st.gyro);
		emit get_data("accel", st.accel);

		m_kalman[0].set_zk(st.accel.x());
		m_kalman[1].set_zk(st.accel.y());
		m_kalman[2].set_zk(st.accel.z());

		m_kalman[3].set_zk(st.gyro.x());
		m_kalman[4].set_zk(st.gyro.y());
		m_kalman[5].set_zk(st.gyro.z());

		Vertex3f kav(Vertex3f(m_kalman[0].xk, m_kalman[1].xk, m_kalman[2].xk));
		Vertex3f kgv(Vertex3f(m_kalman[3].xk, m_kalman[4].xk, m_kalman[5].xk));

		st.accel = kav;
		st.gyro = kgv;

		emit get_data("kalman_accel", kav);

		calc_offsets(st.gyro, st.accel);

		if(!m_is_calc_offset_gyro){
			m_rotate_pos -= st.angular_speed(m_offset_gyro);
		}

		m_telemtries.push_front(st);

		while(m_telemtries.size() >= max_count_telemetry){
			m_telemtries.pop_back();
		}

		m_current_playing_pos += 1;
		if(m_current_playing_pos >= m_downloaded_telemetries.size()){
			m_current_playing_pos = 0;
		}
	}
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
}

QVector3D get_pt_on_line(const QVector3D& p0, const QVector3D& n, double t)
{
	return p0 + n * t;
}

inline double sign(double v1)
{
	return v1 >= 0? 1.0 : -1.0;
}

static inline Vertex3f _V(const Vertex3i& v)
{
	return Vertex3f(v);
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
			Vertex3f tmp(_V(rshift(m_downloaded_telemetries[i].accel, m_shift_accel)) * div_accel);
			glVertex3fv(tmp.data);
		}
		glEnd();

		glColor3f(1, 0, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i< count; i++) {
			Vertex3f tmp(_V(rshift(m_downloaded_telemetries[i].gyro, m_shift_gyro)) * div_gyro);
			glVertex3fv(tmp.data);
		}
		glEnd();
	}

	glLineWidth(4);

	glPushMatrix();

	glRotatef(m_rotate_pos.x(), 1, 0, 0);
	glRotatef(m_rotate_pos.y(), 0, 1, 0);
	glRotatef(m_rotate_pos.z(), 0, 0, 1);

	glColor3f(1.f, 0.2f, 0.2f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glEnd();

	glColor3f(0.2f, 1.f, 0.2f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glEnd();

	glColor3f(0.2f, 0.2f, 1.f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();

	glPopMatrix();

	glLineWidth(1);


	glPointSize(4);

	if(m_telemtries.size()){

		Vertex3f tmp(_V(rshift(m_telemtries[0].gyro, m_shift_gyro)) * div_gyro);

		glColor3f(1, 0.5, 0);
		glBegin(GL_POINTS);
			glVertex3fv(tmp.data);
		glEnd();

		glLineWidth(3);
		draw_line(Vertex3f(), tmp, QColor(255, 128, 0));

		glLineWidth(1);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < m_telemtries.size(); i++){
			StructTelemetry& st = m_telemtries[i];
			float dd = (float)(m_telemtries.size() - i) / m_telemtries.size();
			glColor3f(1 * dd, 0.5 * dd, 0);

			tmp = _V(rshift(st.gyro, m_shift_gyro)) * div_gyro;

			glVertex3fv(tmp.data);
		}
		glEnd();

		glColor3f(0.5, 1, 0);
		glBegin(GL_POINTS);
			tmp = _V(rshift(m_telemtries[0].accel, m_shift_accel)) * div_accel;
			glVertex3fv(tmp.data);
		glEnd();

		glLineWidth(3);
		draw_line(Vertex3f(), tmp, QColor(128, 255, 0));

		glLineWidth(1);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < m_telemtries.size(); i++){
			StructTelemetry& st = m_telemtries[i];
			float dd = (float)(m_telemtries.size() - i) / m_telemtries.size();
			glColor3f(0.5 * dd, 1 * dd, 0);

			tmp = _V(rshift(st.accel, m_shift_accel)) * div_accel;

			glVertex3fv(tmp.data);
		}
		glEnd();
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
		m_count_gyro_offset_data++;
		m_offset_accel += accel;
	}else{
//		gyro -= m_offset_gyro;
//		accel -= m_offset_accel;
	}
}

void GyroData::clear_data()
{
	m_count_gyro_offset_data = 0;
	m_percent_downloaded_data = 1;
	m_offset_gyro = Vertex3f();
	m_offset_accel = Vertex3f();
	m_rotate_pos = Vertex3f();
}

void GyroData::tryParseData(const QByteArray &data)
{
	while(m_telemtries.size() >= max_count_telemetry){
		m_telemtries.pop_back();
	}

	StructTelemetry st;

	QDataStream stream(data);
	st.read_from(stream);

	emit get_data("gyro", st.gyro);
	emit get_data("accel", st.accel);

	m_kalman[0].set_zk(st.accel.x());
	m_kalman[1].set_zk(st.accel.y());
	m_kalman[2].set_zk(st.accel.z());

	m_kalman[3].set_zk(st.gyro.x());
	m_kalman[4].set_zk(st.gyro.y());
	m_kalman[5].set_zk(st.gyro.z());

	Vertex3f kav(Vertex3f(m_kalman[0].xk, m_kalman[1].xk, m_kalman[2].xk));
	Vertex3f kgv(Vertex3f(m_kalman[3].xk, m_kalman[4].xk, m_kalman[5].xk));

	st.accel = kav;
	st.gyro = kgv;

	calc_offsets(st.gyro, st.accel);

	if(!m_is_calc_offset_gyro){
		m_rotate_pos -= st.angular_speed(m_offset_gyro);
	}

	emit get_data("kalman_accel", kav);
	emit get_data("kalman_gyro", kgv);

	WriteLog::instance()->add_data("gyro", st);

	m_telemtries.push_front(st);

	m_time_waiting_telemetry.restart();
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
	m_shift_accel = sxml.get_xml_int("shift_accel");
	m_shift_gyro = sxml.get_xml_int("shift_gyro");
	m_fileName = sxml.get_xml_string("filename");
	m_showing_downloaded_data = sxml.get_xml_int("showing_downloaded_data");
	m_addr = QHostAddress(sxml.get_xml_string("ip"));
	m_port = sxml.get_xml_int("port");

	if(m_addr.isNull()){
		m_addr = QHostAddress("192.168.0.200");
	}
	if(!m_port)
		m_port = 7777;

	double freq = sxml.get_xml_double("freq_playing");
	if(freq){
		set_freq_playing(freq);
	}
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
	sxml.set_dom_value_num("shift_accel", m_shift_accel);
	sxml.set_dom_value_num("shift_gyro", m_shift_gyro);
	sxml.set_dom_value_s("filename", m_fileName);
	sxml.set_dom_value_num("showing_downloaded_data", m_showing_downloaded_data);
	sxml.set_dom_value_s("ip", m_addr.toString());
	sxml.set_dom_value_num("port", m_port);

	double freq = freq_playing();
	sxml.set_dom_value_num("freq_playing", freq);

	sxml.save();
}
