#include "gyrodata.h"

#include <QFile>
#include <QTextStream>
#include <QByteArray>
#include <QStringList>
#include <QColor>
#include <QDebug>

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
#include <Windows.h>
#else
#include <chrono>
#endif

#include <GL/gl.h>

///////////////////////////////

void draw_line(const QVector3D& v1, const QVector3D& v2, const QColor& col = Qt::white)
{
	glColor3ub(col.red(), col.green(), col.blue());

	glBegin(GL_LINES);
	glVertex3d(v1.x(), v1.y(), v1.z());
	glVertex3d(v2.x(), v2.y(), v2.z());
	glEnd();
}

///////////////////////////////
/// \brief GyroData::GyroData
/// \param parent
///
///
GyroData::GyroData(QObject *parent) :
	VirtGLObject(parent)
{
	setType(GYRODATA);
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

	m_fileName = fileName;

	QTextStream tstream(&file);

	m_gyro_data.clear();
	m_accel_data.clear();
	m_temp_data.clear();

	int ind = 0;
	while(!tstream.atEnd()){
		QString line = tstream.readLine();
		line = line.trimmed();
		QStringList sl = line.split(';');

		if(sl.size() < 7){
			qDebug() << "data in line" << ind << "not enough:" << sl.size();
			continue;
		}

		double a1, a2, a3, t, g1, g2, g3;

		a1	= sl[0].toDouble();
		a2	= sl[1].toDouble();
		a3	= sl[2].toDouble();
		t	= sl[3].toDouble();
		g1	= sl[4].toDouble();
		g2	= sl[5].toDouble();
		g3	= sl[6].toDouble();

		m_accel_data.push_back(QVector3D(a1, a2, a3));
		m_gyro_data.push_back(QVector3D(g1, g2, g3));
		m_temp_data.push_back(t);

		ind++;
	}

	normalize_vector(m_accel_data);
	normalize_vector(m_gyro_data);

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
	double mean_radius = 0;
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


void GyroData::init()
{
	openFile("../data/data.csv");
}

QVector3D get_pt_on_line(const QVector3D& p0, const QVector3D& n, double t)
{
	return p0 + n * t;
}

inline double sign(double v1)
{
	return v1 >= 0? 1.0 : -1.0;
}

void GyroData::draw()
{
	glPointSize(3);

	glColor3f(0, 1, 0);
	glBegin(GL_POINTS);
	foreach (QVector3D it, m_accel_data) {
		glVertex3d(it.x(), it.y(), it.z());
	}
	glEnd();

	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	foreach (QVector3D it, m_gyro_data) {
		glVertex3d(it.x(), it.y(), it.z());
	}
	glEnd();

	glPointSize(7);
	glColor3f(1, 1, 0);
	glBegin(GL_POINTS);
	glVertex3f(m_center_accel.x(), m_center_accel.y(), m_center_accel.z());
	glEnd();

	glLineWidth(4);

	glColor3f(1, 0.2, 0.2);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glEnd();

	glColor3f(0.2, 1, 0.2);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glEnd();

	glColor3f(0.2, 0.2, 1);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();

	glLineWidth(1);

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
