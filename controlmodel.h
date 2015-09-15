#ifndef CONTROLMODEL_H
#define CONTROLMODEL_H

#include <QObject>
#include <QTimer>
#include <QTime>

class ControlModel : public QObject
{
	Q_OBJECT
public:
	explicit ControlModel(QObject *parent = 0);

signals:

public slots:
	void on_timeout();

private:
	QTimer m_timer;
};

#endif // CONTROLMODEL_H
