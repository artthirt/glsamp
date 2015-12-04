#include "datachart.h"
#include <QMapIterator>
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QFontMetrics>
#include <QPointF>

using namespace sc;
using namespace vector3_;

DataChart::DataChart(QWidget *parent)
	: QWidget(parent)
{
	m_dt = 10;
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(_on_timeout()));
	m_timer.start(60);
}

void DataChart::clear()
{
	for(QMap<QString, Chart >::iterator it = m_charts.begin(); it != m_charts.end(); it++){
		it.value().clear();
	}
	update();
}

void DataChart::clear(const QString &chart)
{
	if(m_charts.contains(chart))
		m_charts[chart].clear();
}

void DataChart::add_nowatch(const QString &value)
{
	m_nowatch.push_back(value);
}

void DataChart::clear_nowatch()
{
	m_nowatch.clear();
}

void DataChart::_on_timeout()
{

}

void DataChart::_on_put_data(const QString &chart, double value)
{
	if(m_nowatch.contains(chart))
		return;

	m_charts[chart].add_value(value);
}

void DataChart::_on_put_data(const QString &chart, Vector3i value)
{
	if(m_nowatch.contains(chart))
		return;

	QString ch_x = chart + ".x";
	QString ch_y = chart + ".y";
	QString ch_z = chart + ".z";

	m_charts[ch_x].add_value(value.x());
	m_charts[ch_y].add_value(value.y());
	m_charts[ch_z].add_value(value.z());
}

void draw_line(QPainter& painter, const Chart& chart, const QRect& rt, double dt, double minv, double maxv, int max_cnt)
{
	if(!chart.data.size())
		return;

	double dy = rt.height() / (maxv - minv);

	QPolygonF poly;
	QPointF pt_right;
	double val = 0;
	if(chart.data.size() * dt > rt.width()){
		double x = rt.right(), y = rt.height() - 1;
		double offset = (max_cnt - chart.data.size()) * dt;
		if(rt.bottom() - offset > rt.left()){
			for(int i = chart.data.size() - 1; i >= 0 && x >= rt.left(); i--){
				y = rt.bottom() - offset - (chart.data[i] - minv) * dy;

				poly << QPointF(x, y);
				x -= dt;
			}
		}
		pt_right = poly.front();
		val = chart.data.back();
	}else{
		double x = rt.left(), y = rt.height() - 1;
		int ind_min = (max_cnt * dt - rt.width()) / rt.width();
		ind_min = qMax(0, ind_min);
		for(int i = ind_min; i < chart.data.size(); i++){
			y = rt.bottom() - (chart.data[i] - minv) * dy;

			poly << QPointF(x, y);
			x += dt;
		}
		pt_right = poly.back();
		val = chart.data.back();
	}
	if(poly.size()){
		int y = pt_right.y();
		y = (y / 20) * 20;
		pt_right.setY(y);
		pt_right.setX(rt.width() - 20);

		painter.setPen(QPen(QBrush(chart.color), 2));
		painter.drawPolyline(poly);

		painter.drawText(pt_right, QString::number(val, 'f', 1));
	}
}

void DataChart::paintEvent(QPaintEvent *ev)
{
	QPainter painter(this);
	QMapIterator< QString, Chart > it(m_charts);

	double min_value = 0, max_value = 0;
	int max_text_width = 0;
	int max_cnt = 0;

	QFontMetrics fm(font(), this);

	for(QMap< QString, Chart >::iterator it = m_charts.begin(); it != m_charts.end(); it++){
		min_value = qMin(min_value, it.value().min_value);
		max_value = qMax(max_value, it.value().max_value);

		max_cnt = qMax(it.value().data.size(), max_cnt);

		QSize s = fm.size(Qt::TextSingleLine, it.key());
		max_text_width = qMax(max_text_width, s.width());
	}

	QRect rt = rect(), rt_space;
	painter.fillRect(rt, Qt::white);

	painter.setBrush(Qt::NoBrush);
	rt_space = rt.adjusted(20, 20, -20, -20);
	painter.setPen(QPen(QBrush(Qt::black), 3));
	painter.drawRect(rt_space);

	{
		painter.setPen(QPen(QBrush(Qt::lightGray), 1));
		double x = rt_space.left();
		while( x < rt_space.right()){
			painter.drawLine(QPointF(x, rt_space.top()), QPointF(x, rt_space.bottom()));

			x += m_dt;
		}
	}

	painter.setPen(QPen(QBrush(Qt::black), 1));

	for(QMap< QString, Chart >::iterator it = m_charts.begin(); it != m_charts.end(); it++){
		draw_line(painter, it.value(), rt_space, m_dt, min_value, max_value, max_cnt);
	}

	double x = rt_space.right() - max_text_width - 5, y = rt_space.top() + 20;
	for(QMap< QString, Chart >::iterator it = m_charts.begin(); it != m_charts.end(); it++){
		painter.setPen(QPen(it.value().color, 3));
		painter.drawLine(QPointF(x - 10, y), QPointF(x, y));
		painter.setPen(QPen(QBrush(Qt::black), 1));
		painter.drawText(QPointF(x, y), it.key());
		y += 30;
	}

	painter.drawText(QPointF(rt_space.left() - 15, rt_space.top() - 5), QString::number(max_value));
	painter.drawText(QPointF(rt_space.left() - 15, rt_space.bottom() + 20), QString::number(min_value));

	update();
}


QVector<QString> DataChart::nowatch() const
{
	return m_nowatch;
}
