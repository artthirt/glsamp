#ifndef VIRTGLOBJECT_H
#define VIRTGLOBJECT_H

#include <QObject>
#include <QVector3D>

class VirtGLObject: public QObject{
public:
	VirtGLObject(): m_is_watchXY(false), m_is_watch(false){

	}

	/**
	 * @brief init
	 * init object
	 */
	virtual void init() = 0;
	/**
	 * @brief draw
	 * draw primitives
	 */
	virtual void draw() = 0;
	/**
	 * @brief tick
	 * for timer tick
	 */
	virtual void tick() = 0;
	/**
	 * @brief position
	 * return object's position
	 * @return
	 */
	virtual QVector3D position() const = 0;
	/**
	 * @brief is_watchXY
	 * watch in XY plane
	 * @return
	 */
	bool is_watchXY() const { return m_is_watchXY; }
	/**
	 * @brief set_is_watchXY
	 * set watcher for XY plane
	 * @param value
	 */
	void set_is_watchXY(bool value) {
		m_is_watchXY = value;
		if(m_is_watch && value)
			m_is_watch = false;
	}
	/**
	 * @brief is_watch
	 * @return
	 */
	bool is_watch() const { return m_is_watch; }
	/**
	 * @brief set_watch
	 * @param value
	 */
	void set_is_watch(bool value){
		m_is_watch = value;
		if(m_is_watchXY && value)
			m_is_watchXY = false;
	}
private:
	bool m_is_watchXY;
	bool m_is_watch;
};

#endif // VIRTGLOBJECT_H
