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

	bool empty() const{
		return qAbs(mean_radius) < 1e-11;
	}
	void reset(){
		cp = vector3_::Vector3d();
		mean_radius = 0;
		deviation = 0;
	}

	vector3_::Vector3d cp;
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
	/**
	 * @brief set_parameters
	 * @param sts
	 * @param max_pass
	 * @param threshold
	 * @param percent_deviation - percent deviation calculation and mean radius
	 * @return
	 */
	bool set_parameters(const QVector< sc::StructTelemetry > *sts, int max_pass = 100, double threshold = 1e-6,
						double percent_deviation = 12.);
	bool set_parameters(const QVector< vector3_::Vector3d > &sts, int max_pass = 100, double threshold = 1e-6,
						double percent_deviation = 12.);
signals:
	void send_log(const QString& value);

public slots:


private:
	QVector< vector3_::Vector3d > m_analyze_data;

	int m_max_pass;
	int m_pass;
	double m_pass_part_evaluate;
	double m_percent_deviation;
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
	bool search_minmax(const QVector< vector3_::Vector3d >& data, vector3_::Vector3d &min, vector3_::Vector3d &max);
	/**
	 * @brief calc_radius
	 * @param sts
	 * @param p
	 * @param sp
	 */
	void calc_radius(const QVector<vector3_::Vector3d> &sts, const vector3_::Vector3d& p, StructMeanSphere& sp);
	StructMeanSphere circumscribed_sphere_search(QVector<vector3_::Vector3d> &sts, const vector3_::Vector3d& p1, const vector3_::Vector3d& p2,
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
