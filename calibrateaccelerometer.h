#ifndef CALIBRATEACCELEROMETER_H
#define CALIBRATEACCELEROMETER_H

#include <QRunnable>
#include <QVector>

#include "struct_controls.h"

/**
 * @brief The StructPoint struct
 */
struct StructMeanSphere{
	StructMeanSphere(){
		mean_radius = 0;
		deviation = 0;
	}

	bool isNull() const{
		return qAbs(mean_radius) < sc::epsilon;
	}
	void reset(){
		cp = sc::Vector3d();
		mean_radius = 0;
		deviation = 0;
	}

	sc::Vector3d cp;
	double mean_radius;
	double deviation;
};

/**
 * @brief The CalibrateAccelerometer class
 */
class CalibrateAccelerometer : public QObject
{
	Q_OBJECT
public:
	enum STATE_EVALUATE{
		none,
		begin,
		newpass,
		evaluate_mean_radius,
		evaluate_deviation,
		search_min_box,
		end
	};

	explicit CalibrateAccelerometer(QObject *parent = 0);

	STATE_EVALUATE state() const;
	bool is_done() const;
	bool is_progress() const;
	int pass() const;
	double pass_part_evaluate() const;
	double threshold() const;
	StructMeanSphere result() const;
	void evaluate();
	bool set_parameters(const QVector< sc::StructTelemetry > *sts, int max_pass = 100, double threshold = 1e-6);
signals:

public slots:


private:
	const QVector< sc::StructTelemetry > *m_telemetry;

	int m_max_pass;
	int m_pass;
	double m_pass_part_evaluate;
	STATE_EVALUATE m_state;
	double m_threshold;
	StructMeanSphere m_result;

	/**
	 * @brief search_minmax
	 * @param data
	 * @param min
	 * @param max
	 * @return
	 */
	bool search_minmax(const QVector< sc::StructTelemetry >& data, sc::Vector3i& min, sc::Vector3i& max);
	/**
	 * @brief calc_radius
	 * @param sts
	 * @param p
	 * @param sp
	 */
	void calc_radius(const QVector< sc::StructTelemetry >& sts, const sc::Vector3d& p, StructMeanSphere& sp);
	StructMeanSphere circumscribed_sphere_search(const QVector< sc::StructTelemetry >& sts, const sc::Vector3d& p1, const sc::Vector3d& p2,
												 double& dx, double& dy, double& dz);

};

////////////////////////////////////////////////////////

class CalibrateAccelerometerRunnable: public QRunnable
{
public:
	CalibrateAccelerometerRunnable(CalibrateAccelerometer *calibrate);
protected:
	virtual void run();
private:
	CalibrateAccelerometer *m_calibrate;

};

////////////////////////////////////////////////////////

template < typename VT >
inline VT min_v(const VT& v1, const VT& v2)
{
	return VT(
				qMin(v1.x(), v2.x()),
				qMin(v1.y(), v2.y()),
				qMin(v1.z(), v2.z())
				);
}

template < typename VT >
inline VT max_v(const VT& v1, const VT& v2)
{
	return VT(
				qMax(v1.x(), v2.x()),
				qMax(v1.y(), v2.y()),
				qMax(v1.z(), v2.z())
				);
}

#endif // CALIBRATEACCELEROMETER_H
