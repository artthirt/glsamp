#ifndef VIRTGLOBJECT_H
#define VIRTGLOBJECT_H

#include <QObject>
#include <QVector3D>

class VirtGLObject: public QObject{
public:
	enum {
		TYPE_VGL = 0
	};

	VirtGLObject(QObject *parent = NULL):
		QObject(parent),
		m_is_watchXY(false),
		m_is_watch(false)
	  , m_is_enable(true)
	  , m_type(TYPE_VGL)
	{

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
	/**
	 * @brief is_enable
	 * @return
	 */
	bool is_enable() const {
		return m_is_enable;
	}
	/**
	 * @brief set_is_enable
	 * @param value
	 */
	void set_is_enable(bool value){
		m_is_enable = value;
	}
	/**
	 * @brief type
	 * @return
	 */
	int type() const{
		return m_type;
	}

protected:
	/**
	 * @brief setType
	 * @param value
	 */
	void setType(int value){
		m_type = value;
	}

private:
	bool m_is_watchXY;
	bool m_is_watch;
	bool m_is_enable;
	int m_type;
};

#endif // VIRTGLOBJECT_H
