#include "calibrateaccelerometer.h"
#include <QDebug>

using namespace sc;
using namespace vector3_;

CalibrateAccelerometer::CalibrateAccelerometer(QObject *parent) :
	QObject(parent)
  , m_max_pass(100)
  , m_pass(0)
  , m_pass_part_evaluate(0)
  , m_state(none)
  , m_threshold(1e-6)
  , m_percent_deviation(0.12)
{
}

CalibrateAccelerometer::STATE_EVALUATE CalibrateAccelerometer::state() const
{
	return m_state;
}

bool CalibrateAccelerometer::is_done() const
{
	return m_state == end;
}

bool CalibrateAccelerometer::is_progress() const
{
	return m_state != none && m_state != end;
}

int CalibrateAccelerometer::pass() const
{
	return m_pass;
}

double CalibrateAccelerometer::pass_part_evaluate() const
{
	return m_pass_part_evaluate;
}

double CalibrateAccelerometer::threshold() const
{
	return m_threshold;
}

StructMeanSphere CalibrateAccelerometer::result() const
{
	return m_result;
}

bool CalibrateAccelerometer::set_parameters(const QVector<StructTelemetry> *sts, int max_pass, double threshold, double percent_deviation)
{
	if(is_progress())
		return false;

	m_analyze_data.clear();
	foreach (StructTelemetry st, *sts) {
		m_analyze_data.push_back(st.accel);
	}
	m_max_pass = max_pass;
	m_threshold = threshold;
	m_state = none;
	m_percent_deviation = percent_deviation / 100.;

	return true;
}

void CalibrateAccelerometer::evaluate()
{
	m_state = none;

	if(!m_analyze_data.size()){
		return;
	}

	Vector3d p1, p2;
	bool res = search_minmax(m_analyze_data, p1, p2);
	if(!res){
		return;
	}

	m_state = begin;

	StructMeanSphere sphere;

	bool loop = true;
	double dx = 0, dy = 0, dz = 0;
	m_pass = 0;
	while(loop && m_pass < m_max_pass){

		m_state = newpass;

		StructMeanSphere p = circumscribed_sphere_search(m_analyze_data, p1, p2, dx, dy, dz);
		double delta = qAbs(p.deviation - sphere.deviation);
		if(delta < m_threshold){
			loop = false;

		}
		sphere = p;

		dx = qAbs(dx), dy = qAbs(dy), dz = qAbs(dz);

		p1 = p.cp - Vector3d(dx, dy, dz);
		p2 = p.cp + Vector3d(dx, dy, dz);

		QString debug = QString("calibrate: pass=%1; mean_radius=%2; deviation=%3; x=%4; y=%5; z=%6; delta=%7")
				.arg(m_pass++)
				.arg(sphere.mean_radius)
				.arg(sphere.deviation)
				.arg(sphere.cp.x())
				.arg(sphere.cp.y())
				.arg(sphere.cp.z())
				.arg(delta);
		debug += QString("; dx=%1; dy=%2; dz=%3").arg(dx).arg(dy).arg(dz);

		qDebug() << debug;
		emit send_log(debug);

	}

	m_result = sphere;

	m_state = end;
}

bool CalibrateAccelerometer::search_minmax(const QVector< Vector3d >& data, Vector3d& min, Vector3d& max)
{
	if(!data.size())
		return false;
	 Vector3d res_min = data[0];
	 Vector3d res_max = res_min;

	 foreach (Vector3d it, data) {
		res_min = min_v(res_min, it);
		res_max = max_v(res_max, it);
	 }
	 min = res_min;
	 max = res_max;

	 return true;
}

void CalibrateAccelerometer::calc_radius(const QVector< Vector3d >& sts, const Vector3d& p, StructMeanSphere& sp)
{
	m_state = evaluate_mean_radius;

	sp.cp = p;
	QVector< double > radiuses;
	for(int i = 0; i < sts.size(); i++) {
		Vector3d ad(sts[i]);
		ad -= p;
		radiuses.push_back(ad.length());
	}
	double sum = 0;
	foreach (double r, radiuses) {
		sum += r;
	}
	sum /= radiuses.size();

	m_state = evaluate_deviation;

	double deviation = 0;
	foreach (double r, radiuses) {
		deviation += (r - sum) * (r - sum);
	}
	deviation /= radiuses.size();

	sp.mean_radius = sum;
	sp.deviation = sqrt(deviation);
}

StructMeanSphere CalibrateAccelerometer::circumscribed_sphere_search(QVector< Vector3d >& sts, const Vector3d& p1, const Vector3d& p2,
											 double& dx, double& dy, double& dz)
{
	const int count_side = 10;
	const int count = count_side * count_side * count_side;

	int deviat_great_cnt = 0;

	QVector< StructMeanSphere > pts;
	StructMeanSphere res;

	pts.resize(count);

	dx = (p2.x() - p1.x()) / count_side;
	dy = (p2.y() - p1.y()) / count_side;
	dz = (p2.z() - p1.z()) / count_side;

	do{

		deviat_great_cnt = 0;
		Vector3d p = p1;

		for(int i = 0, l = 0; i < count_side; i++){
			p.setY(p1.y());
			for(int j = 0; j < count_side; j++){
				p.setX(p1.x());
				for(int k = 0; k < count_side; k++, l++){
					calc_radius(sts, p, pts[l]);
					p.setX(p.x() + dx);

					m_pass_part_evaluate = (double) l / pts.size();
				}
				p.setY(p.y() + dy);
			}
			p.setZ(p.z() + dz);
		}

		m_state = search_min_box;

		int sid = 0;
		for(int i = 1; i < pts.size(); i++){
			if(pts[sid].deviation > pts[i].deviation){
				sid = i;
			}
		}

		res = pts[sid];
		int all = sts.size();
		for(int j = 0; j < sts.size(); j++){
			Vector3d rd = Vector3d(sts[j]) - pts[sid].cp;
			if(fabs(rd.length() - pts[sid].mean_radius) > m_percent_deviation * pts[sid].mean_radius){
				deviat_great_cnt++;
				sts.remove(j);
			}
		}
		QString debug = QString("calibrate: outlers=%1; count=%2; previous_count=%3")
				.arg(deviat_great_cnt)
				.arg(sts.size())
				.arg(all);
		qDebug() << debug;
		emit send_log(debug);

	}while( deviat_great_cnt > 0);

	return res;
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

CalibrateAccelerometerRunnable::CalibrateAccelerometerRunnable(CalibrateAccelerometer *calibrate)
{
	m_calibrate = calibrate;
}

void CalibrateAccelerometerRunnable::run()
{
	m_calibrate->evaluate();
}
