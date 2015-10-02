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
		return qAbs(mean_radius) < epsilon;
	}
	void reset(){
		cp = Vertex3d();
		mean_radius = 0;
		deviation = 0;
	}

	Vertex3d cp;
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
	bool set_parameters(const QVector<StructTelemetry> *sts, int max_pass = 100, double threshold = 1e-6);
signals:

public slots:


private:
	const QVector< StructTelemetry > *m_telemetry;

	int m_pass;
	int m_max_pass;
	double m_pass_part_evaluate;
	double m_threshold;
	STATE_EVALUATE m_state;
	StructMeanSphere m_result;

	/**
	 * @brief search_minmax
	 * @param data
	 * @param min
	 * @param max
	 * @return
	 */
	bool search_minmax(const QVector< StructTelemetry >& data, Vertex3i& min, Vertex3i& max);
	/**
	 * @brief calc_radius
	 * @param sts
	 * @param p
	 * @param sp
	 */
	void calc_radius(const QVector< StructTelemetry >& sts, const Vertex3d& p, StructMeanSphere& sp);
	StructMeanSphere circumscribed_sphere_search(const QVector< StructTelemetry >& sts, const Vertex3d& p1, const Vertex3d& p2,
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
