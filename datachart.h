#ifndef DATACHART_H
#define DATACHART_H

#include <QWidget>
#include <QVector>
#include <QMap>
#include <QTimer>
#include <struct_controls.h>
#include <QColor>

struct Chart{
	Chart(){
		min_value = max_value = 0;
		max_count = 10000;
		color = QColor((255.0 * rand())/RAND_MAX, (255.0 * rand())/RAND_MAX, (255.0 * rand())/RAND_MAX);
	}

	double min_value;
	double max_value;
	int max_count;
	QVector< double > data;
	QColor color;

	void add_value(double value){
		if(value > max_value || !data.size()){
			max_value = value;
		}
		if(value < min_value || !data.size()){
			min_value = value;
		}
		data.push_back(value);

		if(data.size() >= max_count){
			data.pop_front();
		}
	}
	void clear(){
		min_value = max_value = 0;
		data.clear();
	}
};

class DataChart: public QWidget
{
	Q_OBJECT
public:
	DataChart(QWidget *parent = 0);

	void clear();
	void clear(const QString& chart);

	void add_nowatch(const QString &value);
	void clear_nowatch();
	QVector< QString > nowatch() const;


public slots:
	void _on_timeout();
	void _on_put_data(const QString& chart, double value);
	void _on_put_data(const QString& chart, vector3_::Vector3i value);

signals:

private:
	QTimer m_timer;
	QMap< QString, Chart > m_charts;
	QVector< QString > m_nowatch;
	double m_dt;
	// QWidget interface
protected:
	virtual void paintEvent(QPaintEvent *ev);
};

#endif // DATACHART_H
