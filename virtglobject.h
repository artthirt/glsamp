#ifndef VIRTGLOBJECT_H
#define VIRTGLOBJECT_H

#include <QObject>

class VirtGLObject: public QObject{
public:
	virtual void init() = 0;
	virtual void draw() = 0;
	virtual void tick() = 0;
};

#endif // VIRTGLOBJECT_H
