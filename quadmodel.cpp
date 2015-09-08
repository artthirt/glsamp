#include "quadmodel.h"

#include <Windows.h>
#include <gl/GL.h>

const double wd_lv = 0.1;

const int delta_time = 200;

QuadModel::QuadModel()
{
	m_lever = 1;
	m_mg = 9.8;
	m_max_power = 20;

	m_color = QColor(60, 255, 60, 255);

	for(int i = 0; i < 4; i++) m_engines[i] = 0;
	m_normal = QVector3D(0, 0, 1);

	std::tr1::random_device rd;
	generator = std::tr1::mt19937(rd);

	distribution = std::tr1::normal_distribution<double>(0.005, 0.005);
	distribution_time = std::tr1::normal_distribution<double>(15, 15);

	m_delta_time = delta_time;
	m_time_noise.start();
	m_time_noise_start = m_time_noise.elapsed();

	bool res = connect(&m_timer_noise, SIGNAL(timeout()), this, SLOT(on_timeout_noise()));
	m_timer_noise.start(10);

	generate_engines_rnd();

	reset();
}

double QuadModel::lever() const
{
	return m_lever;
}

void QuadModel::setLever(double value)
{
	m_lever = value;
}

void QuadModel::setAlpha(double value)
{
	m_alpha = value;
}

void QuadModel::setBetha(double value)
{
	m_betha = value;
}

void QuadModel::setGamma(double value)
{
	m_gamma = value;
}

double QuadModel::power() const
{
	double pw = 0;
	for(int i = 0; i < 4; i++){
		pw += m_engines[i] + m_engines_rnd[i];
	}
	return pw;
}

double QuadModel::real_power() const
{
	double pw = 0;
	for(int i = 0; i < 4; i++){
		pw += m_engines[i];
	}
	return pw;
}


void QuadModel::reset()
{
	m_alpha = m_betha = m_gamma = 0;
	for(int i = 0; i < 4; i++) m_engines[i] = 0;
}

void QuadModel::set_max_power(double value)
{
	m_max_power = value;
}

double QuadModel::mg() const
{
	return m_mg;
}

void QuadModel::set_mg(double mg)
{
	m_mg = mg;
}

void QuadModel::add_alpha(double value)
{
	m_alpha += value;
}

void QuadModel::add_betha(double value)
{
	m_betha += value;
}

void QuadModel::add_gamma(double value)
{
	m_gamma += value;
}

void QuadModel::add_power(double value)
{
	for(int i = 0; i < 4; i++){
		m_engines[i] += value;
		if(m_engines[i] > m_max_power){
			m_engines[i] = m_max_power;
		}
		if(m_engines[i] < 0)
			m_engines[i] = 0;
	}
}

void QuadModel::set_power(double value)
{
	for(int i = 0; i < 4; i++){
		m_engines[i] = value + m_engines_rnd[i];
		if(m_engines[i] > m_max_power){
			m_engines[i] = m_max_power;
		}
		if(m_engines[i] < 0)
			m_engines[i] = 0;
	}
}

void QuadModel::setColor(const QColor &color)
{
	m_color = color;
}

QColor QuadModel::color() const
{
	return m_color;
}


void QuadModel::init()
{
	reset();
}

void QuadModel::draw()
{
	glPushMatrix();

	glTranslated(m_position.x(), m_position.y(), m_position.z());

	glColor3d(m_color.redF(), m_color.greenF(), m_color.blueF());

	draw_leter(QVector3D(0, wd_lv, 0), 45);
	draw_leter(QVector3D(0, wd_lv, 0), 135);
	draw_leter(QVector3D(0, wd_lv, 0), -45);
	draw_leter(QVector3D(0, wd_lv, 0), -135);

	glPopMatrix();
}

void QuadModel::tick()
{
	double pw = 0;
	for(int i = 0; i < 4; i++){
		pw += m_engines[i] + m_engines_rnd[i];
	}
	pw -= m_mg;
	if(m_position.z() < 0){
		pw = 0;
		m_position.setZ(0);
	}
	QVector3D dp = pw * m_normal;


	QVector3D tmp_pos = m_position;
	tmp_pos += dp;
	if(tmp_pos.z() < 0){
		tmp_pos.setZ(0);
	}

	m_position = tmp_pos;
}

void QuadModel::on_timeout_noise()
{
	double delta = m_time_noise.elapsed() - m_time_noise_start;
	if(delta > m_delta_time){
		delta = 0;
		m_time_noise.restart();
		m_time_noise_start = m_time_noise.elapsed();
		m_delta_time = delta_time + distribution_time(generator);

		change_engines_rnd();
	}

	delta /= m_delta_time;

	calc_engines_rnd(delta);
}

void QuadModel::change_engines_rnd()
{
	for(int i = 0; i < 4; i++){
		m_cur_engines_rnd[i] = m_next_engines_rnd[i];
		m_next_engines_rnd[i] = distribution(generator);
	}
}

void QuadModel::generate_engines_rnd()
{
	for(int i = 0; i < 4; i++){
		m_cur_engines_rnd[i] = distribution(generator);
		m_next_engines_rnd[i] = distribution(generator);
		m_engines_rnd[i] = m_cur_engines_rnd[i];
	}
}

void QuadModel::calc_engines_rnd(double delta)
{
	double rp = real_power();
	bool zero_rp = qFuzzyIsNull(rp);

	for(int i = 0; i < 4; i++){
		m_engines_rnd[i] = zero_rp? 0 : (m_cur_engines_rnd[i] + delta * (m_next_engines_rnd[i] - m_cur_engines_rnd[i]));
	}
}

typedef double Vector3D[3];

void draw_plane(const Vector3D values[], int w_count, int h_count)
{
//	const int indexes[] = {
//		0, 1, 3,
//		1, 2, 3
//	};

	struct Triangles{
		int A, B, C;
		Triangles(){
			A = B = C = 0;
		}
		Triangles(int A, int B, int C){
			this->A = A;
			this->B = B;
			this->C = C;
		}
	};

	std::vector< Triangles > indexes;

	for(int i = 0; i < w_count - 1; i++){
		for(int j = 0; j < h_count - 1; j++){
			indexes.push_back(Triangles(
				(i)		* w_count + (j),
				(i + 1) * w_count + (j),
				(i + 1)	* w_count + (j + 1)
			));
			indexes.push_back(Triangles(
				(i + 1)	* w_count + (j),
				(i) * w_count + (j + 1),
				(i)	* w_count + (j)
			));
		}
	}

	glBegin(GL_TRIANGLES);
	for(int i = 0; i < indexes.size(); i++){
		const Vector3D &v1 = values[indexes[i].A];
		const Vector3D &v2 = values[indexes[i].B];
		const Vector3D &v3 = values[indexes[i].C];
		glVertex3dv((GLdouble*)&v1);
		glVertex3dv((GLdouble*)&v2);
		glVertex3dv((GLdouble*)&v3);
	}
	glEnd();
}

void QuadModel::draw_leter(QVector3D offset, double angleXY)
{
	const Vector3D plane1[] = {
		{-wd_lv,	0,			wd_lv},
		{-wd_lv,	m_lever,	wd_lv},
		{wd_lv,		m_lever,	wd_lv},
		{wd_lv,		0,			wd_lv}
	};
	const Vector3D plane2[] = {
		{-wd_lv,	0,			-wd_lv},
		{-wd_lv,	m_lever,	-wd_lv},
		{wd_lv,		m_lever,	-wd_lv},
		{wd_lv,		0,			-wd_lv}
	};
	const Vector3D plane3[] = {
		{wd_lv,		0,			-wd_lv},
		{wd_lv,		m_lever,	-wd_lv},
		{wd_lv,		m_lever,	wd_lv},
		{wd_lv,		0,			wd_lv}
	};
	const Vector3D plane4[] = {
		{-wd_lv,	0,			-wd_lv},
		{-wd_lv,	m_lever,	-wd_lv},
		{-wd_lv,	m_lever,	wd_lv},
		{-wd_lv,	0,			wd_lv}
	};

	glPushMatrix();

	glRotated(angleXY, 0, 0, 1);
	glTranslated(offset.x(), offset.y(), offset.z());

	draw_plane(plane1, 2, 2);
	draw_plane(plane2, 2, 2);
	draw_plane(plane3, 2, 2);
	draw_plane(plane4, 2, 2);

	glPopMatrix();
}
