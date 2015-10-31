#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "dialogabout.h"

#include <QFileDialog>
#include "QListWidgetItem"
#include <QLabel>

#include <global.h>
#include <simple_xml.hpp>
#include <writelog.h>

using namespace sc;
using namespace vector3_;

//////////////////////////////////

Q_DECLARE_METATYPE(vector3_::Vector3i)

//////////////////////////////////

const QString xml_config("main.xml");

//////////////////////////////////

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
  , m_available_telemetry(0)
  , m_dataShow(0)
{
	ui->setupUi(this);

	ui->widget->add_object(new QuadModel);
	ui->widget->add_object(new GyroData);

	ui->quadmodel->set_model(qobject_cast<QuadModel*>(ui->widget->item(QuadModel::QUADMODEL)));
	ui->gyrodata->set_model(qobject_cast<GyroData*>(ui->widget->item(GyroData::GYRODATA)));

	m_dataShow = new WndDataShow();

	m_available_telemetry = new QLabel(this);
	m_available_telemetry->setMinimumWidth(200);
	ui->statusBar->addWidget(m_available_telemetry);

	connect(ui->gyrodata->model()->sensorsWork(), SIGNAL(get_data(QString,vector3_::Vector3i)), this, SLOT(_on_put_data(QString,vector3_::Vector3i)));
	connect(ui->gyrodata->model()->sensorsWork(), SIGNAL(get_data(QString,double)), this, SLOT(_on_put_data(QString,double)));

	connect(ui->gyrodata->model(), SIGNAL(add_to_log(QString)), this, SLOT(add_to_log(QString)));
	connect(ui->gyrodata->model()->sensorsWork(), SIGNAL(add_to_log(QString)), this, SLOT(add_to_log(QString)));
	connect(ui->gyrodata->model(), SIGNAL(set_text(QString,QString)), m_dataShow, SLOT(set_text(QString,QString)));
	connect(ui->gyrodata->model()->sensorsWork(), SIGNAL(set_text(QString,QString)), m_dataShow, SLOT(set_text(QString,QString)));

	load_from_xml();
	init_list_objects();

	ui->pte_log->setMaximumBlockCount(1000);

	setWindowState( Qt::WindowMaximized );
}

MainWindow::~MainWindow()
{
	save_to_xml();

	if(m_dataShow)
		delete m_dataShow;

	delete ui;
}

void MainWindow::init_list_objects()
{
	QListWidgetItem *it;

	it = new QListWidgetItem("quadmodel", ui->lw_objects);
	it->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	it->setCheckState(ui->quadmodel->is_enable()? Qt::Checked : Qt::Unchecked);
	it->setData(Qt::UserRole, ui->quadmodel->type());

	it = new QListWidgetItem("sensors data", ui->lw_objects);
	it->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	it->setCheckState(ui->gyrodata->is_enable()? Qt::Checked : Qt::Unchecked);
	it->setData(Qt::UserRole, ui->gyrodata->type());
}

void MainWindow::load_from_xml()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir + xml_config;

	SimpleXML sxml(config_file, SimpleXML::READ);

	if(!sxml.isLoaded())
		return;

	QString str = sxml["state"];
	QByteArray state = QByteArray::fromBase64(str.toLatin1());
	restoreState(state);

	ui->actionShow_log->setChecked(!ui->dw_log->isHidden());
	ui->actionShow_settings->setChecked(!ui->dw_settings->isHidden());

	ui->quadmodel->set_enable(sxml["quadmodel"]);
	ui->gyrodata->set_enable(sxml["gyrodata"]);

	ui->tw_settings->setCurrentIndex(sxml["tab_index"]);
}

void MainWindow::save_to_xml()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir;

	QDir dir(QDir::homePath());

	if(!dir.exists(config_file))
		dir.mkdir(config_file);

	config_file += xml_config;

	SimpleXML sxml(config_file, SimpleXML::WRITE);

	QByteArray state = saveState();
	sxml << "state" << state.toBase64();
	sxml << "quadmodel" << ui->quadmodel->is_enable();
	sxml << "gyrodata" << ui->gyrodata->is_enable();
	sxml << "tab_index" << ui->tw_settings->currentIndex();

}

void MainWindow::closeEvent(QCloseEvent *)
{
	if(m_dataShow){
		m_dataShow->close();
	}
}

void MainWindow::on_lw_objects_itemChanged(QListWidgetItem *item)
{
	QVariant vr = item->data(Qt::UserRole);
	switch (vr.toInt()) {
		case QuadModel::QUADMODEL:
			ui->quadmodel->set_enable(item->checkState() == Qt::Checked? true : false);
			break;
		case GyroData::GYRODATA:
			ui->gyrodata->set_enable(item->checkState() == Qt::Checked? true : false);
			break;
		default:
			break;
	}
}

void MainWindow::_on_put_data(const QString& name , const Vector3i& value)
{
	if(name.contains("accel")){
		ui->widget_graph_accel->_on_put_data(name, value);
	}else{
		if(name.contains("compass"))
			ui->widget_graph_compass->_on_put_data(name, value);
		else
			ui->widget_graph_gyro->_on_put_data(name, value);
	}
}

void MainWindow::_on_put_data(const QString &name, double value)
{
	if(name.contains("accel")){
		ui->widget_graph_accel->_on_put_data(name, value);
	}else{
		if(name.contains("compass"))
			ui->widget_graph_compass->_on_put_data(name, value);
		else
			ui->widget_graph_gyro->_on_put_data(name, value);
	}
}

void MainWindow::on_pb_clear_log_clicked()
{
	ui->pte_log->clear();
}

void MainWindow::add_to_log(const QString &text)
{
	QString line = "[" + QTime::currentTime().toString() + "]  " + text;// + "\r\n";
	//ui->pte_log->moveCursor(QTextCursor::Start);
	//ui->pte_log->insertPlainText(line);
	ui->pte_log->appendPlainText(line);
}

void MainWindow::on_actionAbout_triggered()
{
	DialogAbout dlg;
	dlg.exec();
}

void MainWindow::_on_status_bar_text(const QString &text)
{
	if(m_available_telemetry){
		m_available_telemetry->setText(text);
	}
}

void MainWindow::on_actionShow_data_window_triggered()
{
	if(m_dataShow){
		m_dataShow->show();
	}
}

void MainWindow::on_actionSave_to_Log_triggered(bool checked)
{
	WriteLog::instance()->set_write_log(checked);
}
