#ifndef QUADMODEL_H
#define QUADMODEL_H

#include <virtglobject.h>
#include <QVector>
#include <QPointF>
#include <QTimer>
#include <QTime>
#include <QQuaternion>
#include <QColor>

#include <random>

#include "struct_controls.h"

#if (QT_VERSION <= QT_VERSION_CHECK(5, 0, 0))
#	define QT4
typedef qreal mat_type;
#else
#	define QT5
typedef float mat_type;
#endif

class QuadModel : public VirtGLObject
{
	Q_OBJECT
public:
	enum {
		QUADMODEL = TYPE_VGL + 1
	};

	QuadModel(QObject *parent = NULL);
	/**
	 * @brief lever - lever of quadrocopter model
	 * @return
	 */
	double lever() const;
	/**
	 * @brief setLever
	 * @param value - lever length quadrocopter
	 */
	void setLever(double value);
	/**
	 * @brief power - total power of engine with noise
	 * @return
	 */
	double power() const;
	/**
	 * @brief real_power - total power of engines without noise
	 * @return
	 */
	double real_power() const;
	/**
	 * @brief reset
	 * reset all values. set model in zero position
	 */
	void reset();
	/**
	 * @brief reset_power
	 * reset only power of engines
	 */
	void reset_power();
	/**
	 * @brief set_distribution_parameters
	 * set noise for engines
	 * @param mean
	 * @param sigma
	 */
	void set_distribution_parameters(double mean, double sigma);
	/**
	 * @brief set_max_power
	 * set maximum power
	 * @param value
	 */
	void set_max_power(double value);
	/**
	 * @brief max_power
	 * @return
	 */
	double max_power() { return m_max_power; }
	/**
	 * @brief is_draw_telemetry
	 * @return
	 */
	bool is_draw_telemetry() const { return m_is_draw_telemetry; }
	/**
	 * @brief set_draw_telemetry
	 * set draw telemetry
	 * @param value
	 */
	void set_draw_telemetry(bool value);
	/**
	 * @brief engines
	 * return power of engines without noise
	 * @param index
	 * @return
	 */
	double engines(int index) const {return m_engines[index]; }
	/**
	 * @brief engines_noise
	 * return power of engines with noise
	 * @param index
	 * @return
	 */
	double engines_noise(int index);
	/**
	 * @brief add_power
	 * add power for each engine
	 * @param value
	 */
	void add_power(double value);
	/**
	 * @brief add_power
	 * add power for engine[index]
	 * @param index
	 * @param value
	 */
	void add_power(int index, double value);
	/**
	 * @brief set_power
	 * set current power for each engine
	 * @param value
	 */
	void set_power(double value);
	// VirtGLObject interface
	/**
	 * @brief setColor
	 * @param color
	 */
	void setColor(const QColor &color);
	//
	/**
	 * @brief color
	 * @return
	 */
	QColor color() const;
	/**
	 * @brief set_koeff_fade
	 * koefficient of fade
	 * @param value
	 */
	void set_koeff_fade(double value);
	/**
	 * @brief koeff_fade
	 * koefficient of fade
	 * @return
	 */
	double koeff_fade() const;

	/**
	 * @brief setControl
	 * for control model
	 * @param control
	 */
	void setControl(const StructControls& control);
	/**
	 * @brief telemetry
	 * return current telemetry
	 * @return
	 */
	StructTelemetry telemetry() const;

	/**
	 * @brief is_draw_lever
	 * @return
	 */
	bool is_draw_lever() const;
	/**
	 * @brief set_draw_lever
	 * @param value
	 */
	void set_draw_lever(bool value);

public:
	virtual void init();
	virtual void draw();
	virtual void tick();
	virtual QVector3D position() const;

public slots:
	void on_timeout_noise();

private:
	double m_lever;
	double m_mg;
	double m_max_power;
	double m_koeff;

	bool m_is_draw_lever;

	QColor m_color;
	QVector3D m_normal;
	QVector3D m_course;
	double m_engines[4];

	QVector3D m_delta_speed[2];

	QVector3D m_speed;
	QVector3D m_position;
	double m_rot_speed;
	double m_koeff_fade;

	QTimer m_timer_noise;
	QTime m_time_noise;
	int m_time_noise_start;
	int m_delta_time;

	StructControls m_controls;
	StructTelemetry m_telemetry;

	bool m_is_draw_telemetry;

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
	std::tr1::normal_distribution<double> distribution;
	std::tr1::normal_distribution<double> distribution_time;
	std::tr1::mt19937 generator;
#else
	std::normal_distribution<double> distribution;
	std::normal_distribution<double> distribution_time;
	std::mt19937 generator;
#endif

	QVector3D m_tmp_vv[2];
	QVector3D m_tmp_n[4];
	QVector3D m_tmp_normal, m_tmp_course, m_tmp_vc2;

	void draw_tmp_struct();
	void draw_telemetry();
	void draw_transp_plane(const QMatrix4x4& matrix, const QColor& c);

	QVector< QVector3D > m_trajectory;

	double m_cur_engines_rnd[4];
	double m_next_engines_rnd[4];
	double m_engines_rnd[4];

	void generate_engines_rnd();
	void calc_engines_rnd(double delta);
	void change_engines_rnd();

	void calc_trajectory();

	void draw_lever(QVector3D offset, double angleXY, const QColor col = QColor(0, 255, 0));
};

#endif // QUADMODEL_H
