#include "controlmodel.h"

ControlModel::ControlModel(QObject *parent) :
	QObject(parent)
{
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_timeout()));
	m_timer.start(10);
}

void ControlModel::on_timeout()
{

}
