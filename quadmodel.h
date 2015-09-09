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

class QuadModel : public VirtGLObject
{
	Q_OBJECT
public:
	QuadModel();

	double lever() const;
	void setLever(double value);

	double alpha() const { return m_alpha; }
	void setAlpha(double value);
	double betha() const { return m_betha; }
	void setBetha(double value);
	double gamma() const { return m_gamma; }
	void setGamma( double value);
	double power() const;
	double real_power() const;
	void reset();
	void reset_power();

	double acceleration() const;
	double acceleration_mg() const;

	void set_max_power(double value);
	double max_power() { return m_max_power; }

	double mg() const;
	void set_mg(double mg);

	void set_koeff(double value);
	double koeff() const;

	double engines(int index) const {return m_engines[index]; }
	double engines_noise(int index) { return m_engines[index] + m_engines_rnd[index]; }

	void add_alpha(double value);
	void add_betha(double value);
	void add_gamma(double value);
	void add_power(double value);
	void add_power(int index, double value);
	void set_power(double value);
	// VirtGLObject interface
	void setColor(const QColor &color);
	QColor color() const;

public:
	virtual void init();
	virtual void draw();
	virtual void tick();

public slots:
	void on_timeout_noise();

private:
	double m_alpha;
	double m_betha;
	double m_gamma;
	double m_lever;
	double m_mg;
	double m_max_power;
	double m_koeff;

	double m_acceleration;
	double m_acceleration_mg;

	QColor m_color;
	QVector3D m_normal;
	QVector3D m_course;
	double m_engines[4];

	QVector3D m_delta_speed[2];

	QVector3D m_speed;
	QVector3D m_position;

	QTimer m_timer_noise;
	QTime m_time_noise;
	int m_time_noise_start;
	int m_delta_time;

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
	std::tr1::normal_distribution<double> distribution;
	std::tr1::normal_distribution<double> distribution_time;
	std::tr1::mt19937 generator;
#else
	std::normal_distribution<double> distribution;
	std::normal_distribution<double> distribution_time;
	std::mt19937 generator;
#endif

	QVector< QVector3D > m_trajectory;

	double m_cur_engines_rnd[4];
	double m_next_engines_rnd[4];
	double m_engines_rnd[4];

	void generate_engines_rnd();
	void calc_engines_rnd(double delta);
	void change_engines_rnd();

	void calc_trajectory();

	void draw_leter(QVector3D offset, double angleXY);
};

#endif // QUADMODEL_H
