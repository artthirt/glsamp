#include "calibrateaccelerometer.h"
#include <QDebug>

using namespace sc;

CalibrateAccelerometer::CalibrateAccelerometer(QObject *parent) :
	QObject(parent)
  , m_max_pass(100)
  , m_pass(0)
  , m_pass_part_evaluate(0)
  , m_state(none)
  , m_threshold(1e-6)
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

bool CalibrateAccelerometer::set_parameters(const QVector<StructTelemetry> *sts, int max_pass, double threshold)
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

		qDebug() << "pass: " << m_pass++ << "mean radius: " << sphere.mean_radius << "; deviation: " << sphere.deviation << "; params: " << dx << dy << dz;
		qDebug() << "delta: " << delta;
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

StructMeanSphere CalibrateAccelerometer::circumscribed_sphere_search(const QVector< Vector3d >& sts, const Vector3d& p1, const Vector3d& p2,
											 double& dx, double& dy, double& dz)
{
	const int count_side = 10;
	const int count = count_side * count_side * count_side;

	QVector< StructMeanSphere > pts;
	StructMeanSphere res;

	pts.resize(count);

	dx = (p2.x() - p1.x()) / count_side;
	dy = (p2.y() - p1.y()) / count_side;
	dz = (p2.z() - p1.z()) / count_side;

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
	double deviat_great_cnt = 0;
	for(int j = 0; j < sts.size(); j++){
		Vector3d rd = Vector3d(sts[j]) - pts[sid].cp;
		if(fabs(rd.length() - pts[sid].mean_radius) > 3.0 * pts[sid].deviation){
			deviat_great_cnt++;
		}
	}
	qDebug() << "outlers =" << deviat_great_cnt << "; all =" << sts.size();

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
