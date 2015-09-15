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
	QuadModel(QObject *parent = NULL);

	double lever() const;
	void setLever(double value);

	double power() const;
	double real_power() const;
	void reset();
	void reset_power();

	void set_distribution_parameters(double mean, double sigma);

	void set_max_power(double value);
	double max_power() { return m_max_power; }

	double mg() const;
	void set_mg(double mg);

	void set_koeff(double value);
	double koeff() const;

	bool is_draw_telemetry() const { return m_is_draw_telemetry; }
	void set_draw_telemetry(bool value);

	double engines(int index) const {return m_engines[index]; }
	double engines_noise(int index);

	void add_power(double value);
	void add_power(int index, double value);
	void set_power(double value);
	// VirtGLObject interface
	void setColor(const QColor &color);
	QColor color() const;

	void set_koeff_fade(double value);
	double koeff_fade() const;

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
