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

	connect(&m_tmcalib, SIGNAL(timeout()), this, SLOT(on_timeout_tmcalib()));
	m_tmcalib.setInterval(50);

	init_list_objects();

	ui->lb_filename->setText(m_gyroData.fileName());
	ui->dsb_accel_data->setValue(m_gyroData.divider_accel());
	ui->dsb_div_gyro->setValue(m_gyroData.divider_gyro());
	ui->cb_show_loaded->setChecked(m_gyroData.showing_downloaded_data());
	ui->dsb_frequency_playing->setValue(m_gyroData.freq_playing());
	ui->chb_calibrate_sphere->setChecked(m_gyroData.is_draw_mean_sphere());

	m_available_telemetry = new QLabel(this);
	ui->statusBar->addWidget(m_available_telemetry);

	connect(&m_gyroData, SIGNAL(get_data(QString,Vertex3i)), this, SLOT(on_put_data(QString,Vertex3i)));
	connect(&m_gyroData, SIGNAL(get_data(QString,double)), this, SLOT(on_put_data(QString,double)));

	ui->le_ip_gyro_data->setText(m_gyroData.addr().toString());
	ui->sb_gyro_data->setValue(m_gyroData.port());
	ui->chb_calibratd_data->setChecked(m_gyroData.is_show_calibrated_data());

	ui->widget_pass->setVisible(false);

	connect(&m_gyroData, SIGNAL(add_to_log(QString)), this, SLOT(add_to_log(QString)));

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
	ui->lb_count_value->setText("count: " + QString::number(m_gyroData.count_gyro_offset_data()));
	ui->lb_write_data->setText("count: " + QString::number(m_gyroData.count_write_data()));
}

void MainWindow::on_timeout_tmcalib()
{
	if(m_gyroData.calibrateAccelerometer().is_done()){
		m_tmcalib.stop();
		ui->widget_pass->setVisible(false);
	}
	ui->pb_calibrate->setValue(m_gyroData.calibrateAccelerometer().pass_part_evaluate() * 100.0);
	ui->lb_pass->setText("pass: " + QString::number(m_gyroData.calibrateAccelerometer().pass()));
}

void MainWindow::on_vs_power_valueChanged(int value)
{
	m_model.set_power(value * 0.1);
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
	double v1 = arg1;
	double v2 = ui->dsb_sigma->value();

	m_model.set_distribution_parameters(v1, v2);
}

void MainWindow::on_dsb_sigma_valueChanged(double arg1)
{
	double v1 = ui->dsb_mean->value();
	double v2 = arg1;

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
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir + xml_config;

	SimpleXML sxml(config_file);

	if(!sxml.load())
		return;

	QString str = sxml.get_xml_string("state");
	QByteArray state = QByteArray::fromBase64(str.toLatin1());
	restoreState(state);

	m_model.set_is_enable(sxml.get_xml_int("quadmodel"));
	m_model.set_draw_lever(sxml.get_xml_int("drawlever"));
	m_gyroData.set_is_enable(sxml.get_xml_int("gyrodata"));

	ui->tw_settings->setCurrentIndex(sxml.get_xml_int("tab_index"));

	ui->chb_draw_lever->setChecked(m_model.is_draw_lever());
}

void MainWindow::save_to_xml()
{
	QString config_file = /*QDir::homePath() + */QApplication::applicationDirPath() + "/" + config_dir;

	QDir dir(QDir::homePath());

	if(!dir.exists(config_file))
		dir.mkdir(config_file);

	config_file += xml_config;

	SimpleXML sxml(config_file, true);

	QByteArray state = saveState();
	sxml.set_dom_value_s("state", state.toBase64());
	sxml.set_dom_value_num("quadmodel", m_model.is_enable());
	sxml.set_dom_value_num("gyrodata", m_gyroData.is_enable());
	sxml.set_dom_value_num("drawlever", m_model.is_draw_lever());
	sxml.set_dom_value_num("tab_index", ui->tw_settings->currentIndex());

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

void MainWindow::on_hs_set_end_position_sliderMoved(int position)
{
	m_gyroData.set_end_pos_downloaded_data(position);
}

void MainWindow::on_cb_show_loaded_clicked(bool checked)
{
	m_gyroData.set_showing_downloaded_data(checked);
}

void MainWindow::on_dsb_frequency_playing_valueChanged(double arg1)
{
	m_gyroData.set_freq_playing(arg1);
}

void MainWindow::on_hs_playing_data_valueChanged(int value)
{
//	m_gyroData.set_position_playback(value);
}

void MainWindow::on_pb_play_clicked(bool checked)
{
	if(checked){
		m_gyroData.play();
		if(!m_gyroData.is_play()){
			ui->pb_play->setChecked(false);
		}
	}else{
		m_gyroData.pause();
	}
}

void MainWindow::on_pb_stop_clicked()
{
	m_gyroData.stop();
	ui->pb_play->setChecked(false);
}

void MainWindow::on_pushButton_6_clicked(bool checked)
{
	if(checked){
		m_gyroData.start_calc_offset_gyro();
		ui->lb_count_value->setStyleSheet("background: lightgreen;");
	}else{
		m_gyroData.stop_calc_offset_gyro();
		ui->lb_count_value->setStyleSheet("");
	}
}

void MainWindow::on_pb_reset_clicked()
{
	m_gyroData.stop();
	m_gyroData.reset();
	ui->hs_set_end_position->setValue(m_gyroData.end_pos_downloaded_data());
}

void MainWindow::on_pushButton_9_clicked()
{
	m_gyroData.set_init_position();
}

void MainWindow::on_chb_draw_lever_clicked(bool checked)
{
	m_model.set_draw_lever(checked);
}

void MainWindow::on_pushButton_10_clicked(bool checked)
{
	if(m_gyroData.calibrate()){
		ui->widget_pass->setVisible(true);
		ui->pb_calibrate->setValue(0);
		ui->lb_pass->setText("pass: " + QString::number(m_gyroData.calibrateAccelerometer().pass()));
		m_tmcalib.start();
	}
}

void MainWindow::on_put_data(const QString& name , const Vertex3i& value)
{
	if(name.contains("accel")){
		ui->widget_graph_accel->on_put_data(name, value);
	}else{
		ui->widget_graph_gyro->on_put_data(name, value);
	}
}

void MainWindow::on_put_data(const QString &name, double value)
{
	if(name.contains("accel")){
		ui->widget_graph_accel->on_put_data(name, value);
	}else{
		ui->widget_graph_gyro->on_put_data(name, value);
	}
}

void MainWindow::on_pb_clear_log_clicked()
{
	ui->pte_log->clear();
}

void MainWindow::on_chb_calibrate_sphere_clicked(bool checked)
{
	m_gyroData.set_draw_mean_sphere(checked);
}

void MainWindow::add_to_log(const QString &text)
{
	QString line = "[" + QTime::currentTime().toString() + "]  " + text + "\r\n";
	ui->pte_log->moveCursor(QTextCursor::Start);
	ui->pte_log->insertPlainText(line);
}

void MainWindow::on_chb_calibratd_data_clicked(bool checked)
{
	m_gyroData.set_show_calibrated_data(checked);
}

void MainWindow::on_pushButton_11_clicked(bool checked)
{
	m_gyroData.set_write_data(checked);
	if(checked){
		ui->lb_write_data->setText("write data...");
		ui->lb_write_data->setStyleSheet("background: lightgreen;");
	}else{
		ui->lb_write_data->setStyleSheet("");

	}
}
