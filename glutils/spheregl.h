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
	void draw_sphere(const vector3_::Vector3d& cp, double divider);

private:
	QVector< vector3_::Vector3f > m_vecs_sphere;
	QVector< vector3_::Vector3i > m_inds_sphere;
    double m_divider_accel;
};

#endif // SPHEREGL_H
