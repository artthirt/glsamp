#ifndef SPHEREGL_H
#define SPHEREGL_H

#include <QVector>
#include <struct_controls.h>

class SphereGL
{
public:
	SphereGL();

	void generate_sphere(double radius, double divider);
	void set_divider(double value);
	void draw_sphere(const sc::Vector3d& cp, double divider);

private:
	QVector< sc::Vector3f > m_vecs_sphere;
	QVector< sc::Vector3i > m_inds_sphere;
	double m_divider_accel;
};

#endif // SPHEREGL_H
