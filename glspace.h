#ifndef FORM_H
#define FORM_H

#include <QWidget>
#include <QGLWidget>
#include <QVector>
#include "virtglobject.h"
#include <QVector3D>
#include <QVector2D>
#include <QPointF>

#include <QTimer>

namespace Ui {
class GLSpace;
}

class GLSpace : public QGLWidget
{
	Q_OBJECT

public:
	explicit GLSpace(QWidget *parent = 0);
	~GLSpace();
	/**
	 * @brief add_object
	 * @param obj
	 */
	void add_object(VirtGLObject* obj);
	/**
	 * @brief object
	 * @param index
	 * @return
	 */
	VirtGLObject* object(int index);
	/**
	 * @brief count_objects
	 * @return
	 */
	int count_objects() const;
	/**
	 * @brief setTimeout
	 * @param timeout
	 */
	void setTimeout(int timeout);
	/**
	 * @brief timeout
	 * @return
	 */
	int timeout() const;
	/**
	 * @brief background
	 * @return
	 */
	QColor background() const;
	/**
	 * @brief setBackground
	 * @param color
	 */
	void setBackground(const QColor& color);
private:
	Ui::GLSpace *ui;
	QTimer m_timer;

	QColor m_backround;
	QVector< VirtGLObject* > m_objects;
	QPointF m_mouse_move;
	QPointF m_rotate;
	QVector3D m_translate;
	bool m_mouse_down;
	// QObject interface
	void calc_mouse_move(const QPointF& pos);
	void draw_plane();
public:
	bool event(QEvent *);

	// QGLWidget interface
public slots:
	void updateGL();
	void on_timeout();

protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();
};

#endif // FORM_H
