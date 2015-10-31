#ifndef WNDDATASHOW_H
#define WNDDATASHOW_H

#include <QWidget>
#include <QTimer>
#include <QStandardItemModel>

namespace Ui {
class WndDataShow;
}

class WndDataShow : public QWidget
{
	Q_OBJECT

public:
	explicit WndDataShow(QWidget *parent = 0);
	~WndDataShow();

	void setKeyValue(const QString& key, const QString& value);

signals:

public slots:
	void set_text(const QString &key, const QString text);
	void timeout();

private:
	Ui::WndDataShow *ui;
	QTimer m_timer;
	QStandardItemModel m_model;

	QMap< QString, QString > m_drawing_text;
};

#endif // WNDDATASHOW_H
