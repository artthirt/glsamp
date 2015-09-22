#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include "QListWidgetItem"

#include <global.h>
#include <simple_xml.hpp>

//////////////////////////////////

const QString xml_config("main.xml");

//////////////////////////////////

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
  , m_available_telemetry(0)
{
	ui->setupUi(this);

	ui->widget->add_object(&m_model);
	ui->widget->add_object(&m_gyroData);

	load_from_xml();

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_timeout()));
	m_timer.start(100);
	connect(&m_timer_cfg, SIGNAL(timeout()), this, SLOT(on_timeout_cfg()));
	m_timer_cfg.start(300);

	init_list_objects();

	ui->lb_filename->setText(m_gyroData.fileName());
	ui->dsb_accel_data->setValue(m_gyroData.divider_accel());
	ui->dsb_div_gyro->setValue(m_gyroData.divider_gyro());
	ui->sb_rshift_accel->setValue(m_gyroData.shift_accel());
	ui->sb_rshift_gyro->setValue(m_gyroData.shift_gyro());
	ui->cb_show_loaded->setChecked(m_gyroData.showing_downloaded_data());

	m_available_telemetry = new QLabel(this);
	ui->statusBar->addWidget(m_available_telemetry);

	connect(&m_gyroData, SIGNAL(get_data(QString,Vertex3i)), ui->widget_graph, SLOT(on_put_data(QString,Vertex3i)));
	connect(&m_gyroData, SIGNAL(get_data(QString,double)), ui->widget_graph, SLOT(on_put_data(QString,double)));

	ui->widget_graph->add_nowatch("accel");

	setWindowState( Qt::WindowMaximized );
}

MainWindow::~MainWindow()
{
	save_to_xml();

	delete ui;
}

void MainWindow::on_pb_zero_clicked()
{
	m_model.reset();
}

/*	scheme of engines
 *		0	 2
 *		 \	/
 *		  --
 *		 /  \
 *		3    1
*/

void MainWindow::on_pb_left_front_clicked()
{
	double val = ui->dsb_power->value();
	m_model.add_power(0, val);
}

void MainWindow::on_pb_right_back_clicked()
{
	double val = ui->dsb_power->value();
	m_model.add_power(1, val);
}

void MainWindow::on_pb_right_front_clicked()
{
	double val = ui->dsb_power->value();
	m_model.add_power(2, val);
}

void MainWindow::on_pb_left_back_clicked()
{
	double val = ui->dsb_power->value();
	m_model.add_power(3, val);
}

void MainWindow::on_pb_down_clicked()
{
	double val = ui->dsb_power->value();
	m_model.add_power(-val);
}

void MainWindow::on_pb_up_clicked()
{
	double val = ui->dsb_power->value();
	m_model.add_power(val);
}

void MainWindow::on_timeout()
{
	ui->label_power->setText(QString::number(m_model.power()));

	QString txt;
	for(int i = 0; i < 4; i++){
		txt += QString::number(m_model.engines(i), 'f', 3) + "; ";
	}
	ui->label_power_engines->setText(txt);

	txt = "";
	for(int i = 0; i < 4; i++){
		txt += QString::number(m_model.engines_noise(i), 'f', 3) + "; ";
	}
	ui->label_power_engines_noise->setText(txt);
}

void MainWindow::on_timeout_cfg()
{
	if(m_available_telemetry){
		if(m_gyroData.is_available_telemetry()){
			m_available_telemetry->setText("telemetry received");
		}else{
			m_available_telemetry->setText("telemetry not received");
		}
	}

	if(m_gyroData.is_play()){
		ui->hs_playing_data->setValue(m_gyroData.percent_position());
	}
}

void MainWindow::on_vs_power_valueChanged(int value)
{
	m_model.set_power(ui->vs_power->value() * 0.1);
}

void MainWindow::on_pb_zero_2_clicked()
{
	m_model.reset_power();
}

void MainWindow::on_cb_watch_clicked(bool checked)
{
	m_model.set_is_watchXY(checked);
}

void MainWindow::on_cb_watch_gl_clicked(bool checked)
{
	m_model.set_is_watch(checked);
}

void MainWindow::on_pushButton_2_clicked()
{
	double val = ui->dsb_power->value();

	m_model.add_power(1, val);
	m_model.add_power(3, val);
}

void MainWindow::on_pushButton_clicked()
{
	double val = ui->dsb_power->value();

	m_model.add_power(0, val);
	m_model.add_power(2, val);
}

void MainWindow::on_pushButton_3_clicked()
{
	double val = ui->dsb_power->value();

	m_model.add_power(2, val);
	m_model.add_power(1, val);
}

void MainWindow::on_pushButton_4_clicked()
{
	double val = ui->dsb_power->value();

	m_model.add_power(0, val);
	m_model.add_power(3, val);
}

void MainWindow::on_dsb_mean_valueChanged(double arg1)
{
	double v1 = ui->dsb_mean->value();
	double v2 = ui->dsb_sigma->value();

	m_model.set_distribution_parameters(v1, v2);
}

void MainWindow::on_dsb_sigma_valueChanged(double arg1)
{
	double v1 = ui->dsb_mean->value();
	double v2 = ui->dsb_sigma->value();

	m_model.set_distribution_parameters(v1, v2);
}

void MainWindow::init_list_objects()
{
	QListWidgetItem *it;

	it = new QListWidgetItem("quadmodel", ui->lw_objects);
	it->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	it->setCheckState(m_model.is_enable()? Qt::Checked : Qt::Unchecked);
	it->setData(Qt::UserRole, m_model.type());

	it = new QListWidgetItem("gyro data", ui->lw_objects);
	it->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	it->setCheckState(m_gyroData.is_enable()? Qt::Checked : Qt::Unchecked);
	it->setData(Qt::UserRole, m_gyroData.type());
}

void MainWindow::load_from_xml()
{
	QString config_file = QDir::homePath() + config_dir + xml_config;

	SimpleXML sxml(config_file);

	if(!sxml.load())
		return;

	QString str = sxml.get_xml_string("state");
	QByteArray state = QByteArray::fromBase64(str.toLatin1());
	restoreState(state);

	m_model.set_is_enable(sxml.get_xml_int("quadmodel"));
	m_gyroData.set_is_enable(sxml.get_xml_int("gyrodata"));
}

void MainWindow::save_to_xml()
{
	QString config_file = QDir::homePath() + config_dir;

	QDir dir(QDir::homePath());

	if(!dir.exists(config_file))
		dir.mkdir(config_file);

	config_file += xml_config;

	SimpleXML sxml(config_file, true);

	QByteArray state = saveState();
	sxml.set_dom_value_s("state", state.toBase64());
	sxml.set_dom_value_num("quadmodel", m_model.is_enable());
	sxml.set_dom_value_num("gyrodata", m_gyroData.is_enable());

	sxml.save();

}

void MainWindow::on_lw_objects_itemChanged(QListWidgetItem *item)
{
	QVariant vr = item->data(Qt::UserRole);
	switch (vr.toInt()) {
		case QuadModel::QUADMODEL:
			m_model.set_is_enable(item->checkState() == Qt::Checked? true : false);
			break;
		case GyroData::GYRODATA:
			m_gyroData.set_is_enable(item->checkState() == Qt::Checked? true : false);
			break;
		default:
			break;
	}
}

void MainWindow::on_pushButton_5_clicked()
{
	QFileDialog dlg;

	dlg.setNameFilter("*.csv");

	if(dlg.exec()){
		m_gyroData.openFile(dlg.selectedFiles()[0]);

		ui->lb_filename->setText(m_gyroData.fileName());
	}
}

void MainWindow::on_pushButton_6_clicked()
{
	m_gyroData.recalc_accel(ui->dsb_threshold->value());
}

void MainWindow::on_pushButton_7_clicked()
{
	m_gyroData.send_start_to_net(QHostAddress(ui->le_ip_gyro_data->text()), ui->sb_gyro_data->value());
}

void MainWindow::on_pushButton_8_clicked()
{
	m_gyroData.send_stop_to_net(QHostAddress(ui->le_ip_gyro_data->text()), ui->sb_gyro_data->value());
}

void MainWindow::on_dsb_div_gyro_valueChanged(double arg1)
{
	m_gyroData.set_divider_gyro(arg1);
}

void MainWindow::on_dsb_accel_data_valueChanged(double arg1)
{
	m_gyroData.set_divider_accel(arg1);
}

void MainWindow::on_sb_rshift_gyro_valueChanged(int arg1)
{
	m_gyroData.set_shift_gyro(arg1);
}

void MainWindow::on_sb_rshift_accel_valueChanged(int arg1)
{
	m_gyroData.set_shift_accel(arg1);
}

void MainWindow::on_hs_set_end_position_sliderMoved(int position)
{
	m_gyroData.set_end_pos_downloaded_data(position);
}

void MainWindow::on_pushButton_9_clicked()
{
	m_gyroData.play();
}

void MainWindow::on_pushButton_10_clicked()
{
	m_gyroData.stop();
}

void MainWindow::on_cb_show_loaded_clicked(bool checked)
{
	m_gyroData.set_showing_downloaded_data(checked);
}
