#include "wnddatashow.h"
#include "ui_wnddatashow.h"

#include <QMapIterator>
#include <QMouseEvent>

WndDataShow::WndDataShow(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::WndDataShow)
{
	ui->setupUi(this);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(timeout()));
	m_timer.start(100);

	ui->tw_dataShow->setModel(&m_model);

	m_model.setColumnCount(2);

	m_model.setHorizontalHeaderItem(0, new QStandardItem("Key"));
	m_model.setHorizontalHeaderItem(1, new QStandardItem("Value"));
}

WndDataShow::~WndDataShow()
{
	delete ui;
}

void WndDataShow::setKeyValue(const QString &key, const QString &value)
{
	for(int i = 0; i < m_model.rowCount(); i++){
		QStandardItem* index = m_model.item(i, 0);
		if(index->text() == key){
			QStandardItem* valueI = m_model.item(i, 1);
			valueI->setText(value);
			return;
		}
	}
	QList< QStandardItem* > list;
	list << new QStandardItem(key) << new QStandardItem(value);
	m_model.appendRow(list);
}

void WndDataShow::set_text(const QString &key, const QString text)
{
	m_drawing_text[key] = text;
}

void WndDataShow::timeout()
{
	QMapIterator<QString, QString> it(m_drawing_text);

	while(it.hasNext()){
		it.next();
		setKeyValue(it.key(), it.value());
	}
}

void WndDataShow::on_checkBox_clicked(bool checked)
{
	int flags = windowFlags();
	if(checked){
		flags |= Qt::WindowStaysOnTopHint;
	}else{
		flags &= (~Qt::WindowStaysOnTopHint);
	}
	setWindowFlags((Qt::WindowFlags)flags);
	show();
}


void WndDataShow::mousePressEvent(QMouseEvent *event)
{
}
