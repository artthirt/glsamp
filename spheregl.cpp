#include "spheregl.h"

using namespace sc;
using namespace vector3_;

#include <GL/gl.h>
#include <GL/glu.h>


//////////////////////////////////////////////

template < typename T >
inline T get_pt_sphere(double id, double jd, double R)
{
	double x = R * sin(id * 2 * M_PI) * sin(jd * M_PI);
	double y = R * cos(id * 2 * M_PI) * sin(jd * M_PI);
	double z = R * cos(jd * M_PI);
	return T(x, y, z);
}

//////////////////////////////////////////////
/// \brief SphereGL::SphereGL
///
SphereGL::SphereGL()
{

}

void SphereGL::generate_sphere(double radius, double divider)
{
	if(radius == 0)
		return;

	const int count = 32;
	double R = radius / divider;

	m_vecs_sphere.clear();
	m_inds_sphere.clear();

	for(int i = 0; i < count; i++){
		double id = (double)i / (count - 1);
		for(int j = 0; j < count; j++){
			double jd = (double)j / (count - 1);
			m_vecs_sphere.push_back(get_pt_sphere<Vector3f>(id, jd, R));
		}
	}

	for(int i = 0; i < count - 1; i++){
		for(int j = 0; j < count - 1; j++){
			int A = i * count + j;
			int B = (i + 1) * count + j;
			int C = i * count + j + 1;
			m_inds_sphere.push_back(Vector3i(A, B, C));
		}
	}

	glVertexPointer(3, GL_FLOAT, sizeof(Vector3f), m_vecs_sphere.data()->data);
}


void SphereGL::draw_sphere(const Vector3d &cp, double divider)
{
	if(!m_inds_sphere.size())
		return;

	glPushMatrix();

	Vector3d cpi(cp);
	cpi *= 1.0 / divider;

	glTranslated(cpi.x(), cpi.y(), cpi.z());

	glColor3f(1, 1, 1);

	int cnt = m_inds_sphere.size() * 3;
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawElements(GL_LINE_STRIP, cnt, GL_UNSIGNED_INT, m_inds_sphere.data());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}
