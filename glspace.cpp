#include "glspace.h"
#include "ui_glspace.h"

#include <QMouseEvent>
#include <QWheelEvent>

#if (_MSC_VER >= 1500 && _MSC_VER <= 1600)
#include <Windows.h>
#else
#endif

#include <GL/gl.h>
#include <GL/glu.h>

GLSpace::GLSpace(QWidget *parent) :
	QGLWidget(parent),
	ui(new Ui::GLSpace)
{
	ui->setupUi(this);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_timeout()));
	m_timer.start(20);

	m_backround = QColor(0, 0, 0, 0);
	m_translate.setZ(-2);
}

GLSpace::~GLSpace()
{
	delete ui;
}

void GLSpace::add_object(VirtGLObject *obj)
{
	if(!m_objects.contains(obj)){
		obj->init();
		m_objects.push_back(obj);
	}
}

VirtGLObject *GLSpace::object(int index)
{
	return m_objects[index];
}

int GLSpace::count_objects() const
{
	return m_objects.size();
}

void GLSpace::setTimeout(int timeout)
{
	m_timer.setInterval(timeout);
}

int GLSpace::timeout() const
{
	return m_timer.interval();
}

QColor GLSpace::background() const
{
	return m_backround;
}

void GLSpace::setBackground(const QColor &color)
{
	m_backround = color;
}

void GLSpace::calc_mouse_move(const QPointF &pos)
{
	if(!m_mouse_down)
		return;
	QPointF delta = pos - m_mouse_move;
	m_rotate += delta * 0.5;
}

void GLSpace::draw_plane()
{

	glColor3f(1, 1, 1);

	const int count = 30;
	const double width = 100;

	glBegin(GL_LINES);

	for(int i = 0; i < count; i++){
		glVertex3d(-width/2.0,
				   -width/2.0 + width * i/count,
				   0);
		glVertex3d(-width/2.0 + width,
				   -width/2.0 + width * i/count,
				   0);
	}
	glEnd();

	glBegin(GL_LINES);
	for(int i = 0; i < count; i++){
		glVertex3d(-width/2.0 + width* i/count,
				   -width/2.0,
				   0);
		glVertex3d(-width/2.0 + width* i/count,
				   -width/2.0 + width,
				   0);
	}

	glEnd();
}

bool GLSpace::event(QEvent *ev)
{
	QMouseEvent* mev;
	QWheelEvent* wev;
	double z;

	switch (ev->type()) {
		case QEvent::MouseButtonPress:
			mev = dynamic_cast<QMouseEvent*>(ev);
			m_mouse_down = true;
			m_mouse_move = mev->localPos();
			break;
		case QEvent::MouseButtonRelease:
			mev = dynamic_cast<QMouseEvent*>(ev);
			m_mouse_down = false;
			m_mouse_move = mev->localPos();
			break;
		case QEvent::MouseMove:
			mev = dynamic_cast<QMouseEvent*>(ev);
			calc_mouse_move(mev->localPos());
			m_mouse_move = mev->localPos();
			break;
		case QEvent::Wheel:
			wev = dynamic_cast< QWheelEvent* >(ev);
			z = m_translate.z();
			m_translate.setZ(z + wev->delta() * 0.01);
			break;
		default:
			break;
	}

	return QGLWidget::event(ev);
}

void GLSpace::updateGL()
{
	QGLWidget::updateGL();
}

void GLSpace::on_timeout()
{
	foreach (VirtGLObject* obj, m_objects) {
		obj->tick();
	}

	updateGL();
}

void GLSpace::initializeGL()
{
	glEnable(GL_DEPTH);
	glDepthFunc(GL_LESS);

	foreach (VirtGLObject* obj, m_objects) {
		obj->init();
	}
}

void GLSpace::resizeGL(int w, int h)
{
	double ar = 1.0 * w/h;

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.5 * ar, 0.5 * ar, -0.5, 0.5, 1, 1000);
	glMatrixMode(GL_MODELVIEW);
}

void GLSpace::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(m_backround.redF(), m_backround.greenF(), m_backround.blueF(), m_backround.alphaF());

	glLoadIdentity();
	glTranslatef(m_translate.x(), m_translate.y(), m_translate.z());

	glRotated(m_rotate.x(), 0, 1, 0);
	glRotated(m_rotate.y(), 1, 0, 0);

	draw_plane();

	foreach (VirtGLObject* obj, m_objects) {
		obj->draw();
	}
}
