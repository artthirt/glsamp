#include "gyrodatawidget.h"
#include "ui_gyrodatawidget.h"

#include <QFileDialog>

GyroDataWidget::GyroDataWidget(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::GyroDataWidget)
  , m_model(0)
{
	ui->setupUi(this);

	connect(&m_timer_cfg, SIGNAL(timeout()), this, SLOT(_on_timeout_cfg()));
	m_timer_cfg.start(300);

	connect(&m_tmcalib, SIGNAL(timeout()), this, SLOT(_on_timeout_tmcalib()));
	m_tmcalib.setInterval(50);

	ui->lb_work_compass->setVisible(false);
}

GyroDataWidget::~GyroDataWidget()
{
	delete ui;
}

void GyroDataWidget::set_model(GyroData *model)
{
	m_model = model;

	init_model();
}

void GyroDataWidget::init_model()
{
	if(!m_model)
		return;

	ui->le_ip_gyro_data->setText(m_model->addr().toString());
	ui->sb_gyro_data->setValue(m_model->port());
	ui->chb_calibratd_data->setChecked(m_model->is_show_calibrated_data());

	ui->lb_filename->setText(m_model->fileName());
	ui->dsb_accel_data->setValue(m_model->divider_accel());
	ui->dsb_div_gyro->setValue(m_model->divider_gyro());
	ui->cb_show_loaded->setChecked(m_model->showing_downloaded_data());
	ui->dsb_frequency_playing->setValue(m_model->freq_playing());
	ui->chb_calibrate_sphere->setChecked(m_model->is_draw_mean_sphere());
	ui->chb_recorded_data->setChecked(m_model->is_show_recorded_data());

	ui->widget_pass->setVisible(false);

	QString val = QString("center: %1; radius: %2; deviation: %3")
			.arg(m_model->sensorsWork()->mean_sphere().cp)
			.arg(m_model->sensorsWork()->mean_sphere().mean_radius)
			.arg(m_model->sensorsWork()->mean_sphere().deviation);
	ui->lb_values_accel->setText(val);

	val = QString("center: %1; radius: %2; deviation: %3")
			.arg(m_model->sensorsWork()->mean_sphere_compass().cp)
			.arg(m_model->sensorsWork()->mean_sphere_compass().mean_radius)
			.arg(m_model->sensorsWork()->mean_sphere_compass().deviation);
	ui->lb_values->setText(val);

}

void GyroDataWidget::_on_timeout_cfg()
{
	if(!m_model || !m_model->sensorsWork())
		return;
	if(m_model->sensorsWork()->is_available_telemetry()){
		emit set_status_bar_text("telemetry received");
	}else{
		emit set_status_bar_text("telemetry not received");
	}

	if(m_model->is_play()){
		ui->hs_playing_data->setValue(m_model->percent_position());
	}
	ui->lb_count_value->setText("count: " + QString::number(m_model->sensorsWork()->count_gyro_offset_data()));
	ui->lb_write_data->setText("count: " + QString::number(m_model->count_write_data()));

	const QString stsh("background: lightgreen;");
	if(m_model->sensorsWork()->is_exists_value(SensorsWork::POS_0)){
		ui->pb_zero_pos->setStyleSheet(stsh);
	}
	if(m_model->sensorsWork()->is_exists_value(SensorsWork::POS_90)){
		ui->pb_ninety_->setStyleSheet(stsh);
	}
	if(m_model->sensorsWork()->is_exists_value(SensorsWork::POS_180)){
		ui->pb_invert_pos->setStyleSheet(stsh);
	}
}

void GyroDataWidget::_on_timeout_tmcalib()
{
	if(!m_model || !m_model->sensorsWork())
		return;
	if(m_model->sensorsWork()->calibrate_thread().is_done()){
		m_tmcalib.stop();
		ui->widget_pass->setVisible(false);
		ui->lb_work_compass->setVisible(false);

		switch (m_model->sensorsWork()->typeOfCalibrate()) {
			case SensorsWork::Accelerometer:
				{
					QString val = QString("center: %1; radius: %2; deviation: %3")
							.arg(m_model->sensorsWork()->mean_sphere().cp)
							.arg(m_model->sensorsWork()->mean_sphere().mean_radius)
							.arg(m_model->sensorsWork()->mean_sphere().deviation);
					ui->lb_values_accel->setText(val);
				}
				break;
			case SensorsWork::Compass:
				{
					QString val = QString("center: %1; radius: %2; deviation: %3")
							.arg(m_model->sensorsWork()->mean_sphere_compass().cp)
							.arg(m_model->sensorsWork()->mean_sphere_compass().mean_radius)
							.arg(m_model->sensorsWork()->mean_sphere_compass().deviation);
					ui->lb_values->setText(val);
				}
				break;
			default:
				break;
		}
	}

	ui->pb_calibrate->setValue(m_model->sensorsWork()->calibrate_thread().pass_part_evaluate() * 100.0);
	ui->lb_pass->setText("pass: " + QString::number(m_model->sensorsWork()->calibrate_thread().pass()));
}

///////////////////////////////////
///////////////////////////////////

void GyroDataWidget::on_pushButton_5_clicked()
{
	if(!m_model)
		return;
	QFileDialog dlg;

	dlg.setNameFilter("*.csv");

	if(dlg.exec()){
		m_model->openFile(dlg.selectedFiles()[0]);

		ui->lb_filename->setText(m_model->fileName());
	}
}

void GyroDataWidget::on_pushButton_7_clicked()
{
	if(!m_model)
		return;

	m_model->send_start_to_net();
}

void GyroDataWidget::on_pushButton_8_clicked()
{
	if(!m_model)
		return;
	m_model->send_stop_to_net();
}

void GyroDataWidget::on_dsb_div_gyro_valueChanged(double arg1)
{
	if(!m_model)
		return;
	m_model->set_divider_gyro(arg1);
}

void GyroDataWidget::on_dsb_accel_data_valueChanged(double arg1)
{
	if(!m_model)
		return;
	m_model->set_divider_accel(arg1);
}

void GyroDataWidget::on_hs_set_end_position_sliderMoved(int position)
{
	if(!m_model)
		return;
	m_model->set_end_pos_downloaded_data(position);
}

void GyroDataWidget::on_cb_show_loaded_clicked(bool checked)
{
	if(!m_model)
		return;
	m_model->set_showing_downloaded_data(checked);
}

void GyroDataWidget::on_dsb_frequency_playing_valueChanged(double arg1)
{
	if(!m_model)
		return;
	m_model->set_freq_playing(arg1);
}

void GyroDataWidget::on_hs_playing_data_valueChanged(int value)
{
//	m_model->set_position_playback(value);
}

void GyroDataWidget::on_pb_play_clicked(bool checked)
{
	if(!m_model)
		return;
	if(checked){
		m_model->play();
		if(!m_model->is_play()){
			ui->pb_play->setChecked(false);
		}
	}else{
		m_model->pause();
	}
}

void GyroDataWidget::on_pb_stop_clicked()
{
	if(!m_model)
		return;
	m_model->stop();
	ui->pb_play->setChecked(false);
}

void GyroDataWidget::on_pushButton_6_clicked(bool checked)
{
	if(!m_model)
		return;
	if(checked){
		m_model->sensorsWork()->start_calc_offset_gyro();
		ui->lb_count_value->setStyleSheet("background: lightgreen;");
	}else{
		m_model->sensorsWork()->stop_calc_offset_gyro();
		ui->lb_count_value->setStyleSheet("");
	}
}

void GyroDataWidget::on_pb_reset_clicked()
{
	if(!m_model)
		return;
	m_model->stop();
	m_model->reset();
	ui->hs_set_end_position->setValue(m_model->end_pos_downloaded_data());
}

void GyroDataWidget::on_pushButton_9_clicked()
{
	m_model->set_init_position();
}

void GyroDataWidget::on_pushButton_10_clicked(bool checked)
{
	if(!m_model)
		return;
	if(m_model->calibrate_accelerometer()){
		ui->widget_pass->setVisible(true);
		ui->pb_calibrate->setValue(0);
		ui->lb_pass->setText("pass: " + QString::number(m_model->sensorsWork()->calibrate_thread().pass()));
		m_tmcalib.start();
	}
}

void GyroDataWidget::on_chb_calibrate_sphere_clicked(bool checked)
{
	if(!m_model)
		return;
	m_model->set_draw_mean_sphere(checked);
}

void GyroDataWidget::on_chb_calibratd_data_clicked(bool checked)
{
	if(!m_model)
		return;
	m_model->set_show_calibrated_data(checked);
}

void GyroDataWidget::on_pushButton_11_clicked(bool checked)
{
	if(!m_model)
		return;
	m_model->set_write_data(checked);
	if(checked){
		ui->lb_write_data->setText("write data...");
		ui->lb_write_data->setStyleSheet("background: lightgreen;");
	}else{
		ui->lb_write_data->setStyleSheet("");

	}
}

void GyroDataWidget::on_pb_save_calibration_clicked()
{
	if(m_model){
		m_model->sensorsWork()->save_calibrate();
	}
}

void GyroDataWidget::on_pushButton_clicked()
{
	if(m_model){
		m_model->reset_trajectory();
	}
}

void GyroDataWidget::on_pb_zero_pos_clicked()
{
	ui->pb_zero_pos->setStyleSheet("");
	if(m_model){
		m_model->sensorsWork()->set_start_calc_pos(SensorsWork::POS_0);
	}
}

void GyroDataWidget::on_pb_ninety__clicked()
{
	ui->pb_ninety_->setStyleSheet("");
	if(m_model){
		m_model->sensorsWork()->set_start_calc_pos(SensorsWork::POS_90);
	}
}

void GyroDataWidget::on_pb_invert_pos_clicked()
{
	ui->pb_invert_pos->setStyleSheet("");
	if(m_model){
		m_model->sensorsWork()->set_start_calc_pos(SensorsWork::POS_180);
	}
}

void GyroDataWidget::on_pb_add_to_pool_clicked(bool checked)
{
	if(m_model){
		m_model->add_to_pool(checked);
	}
}

void GyroDataWidget::on_pb_cancel_clicked()
{
	if(m_model){
		m_model->cancel_write();
	}
}

void GyroDataWidget::on_pb_load_calibration_clicked()
{
	if(m_model){
		m_model->sensorsWork()->load_calibrate();
	}
}

void GyroDataWidget::on_chb_recorded_data_clicked(bool checked)
{
	if(m_model){
		m_model->show_recorded_data(checked);
	}
}

void GyroDataWidget::on_pb_write_log_clicked()
{
	if(m_model){
		m_model->log_recorded_data();
	}
}

void GyroDataWidget::on_pb_calibrate_compass_clicked()
{
	if(!m_model)
		return;

	if(m_model->calibrate_compass()){
		ui->lb_work_compass->setVisible(true);
		ui->widget_pass->setVisible(true);
		ui->pb_calibrate->setValue(0);
		ui->lb_pass->setText("pass: " + QString::number(m_model->sensorsWork()->calibrate_thread().pass()));
		m_tmcalib.start();
	}
}

void GyroDataWidget::on_pb_reset_compass_vcalibrate_clicked()
{
	if(m_model){
		m_model->sensorsWork()->reset_calibration_compass();
	}
}

void GyroDataWidget::on_le_ip_gyro_data_returnPressed()
{
	if(!m_model)
		return;
	m_model->set_address(QHostAddress(ui->le_ip_gyro_data->text()), ui->sb_gyro_data->value());
}

void GyroDataWidget::on_sb_gyro_data_valueChanged(int arg1)
{
	if(!m_model)
		return;
	m_model->set_address(QHostAddress(ui->le_ip_gyro_data->text()), ui->sb_gyro_data->value());
}

void GyroDataWidget::on_pb_gpio_send_clicked()
{
	if(!m_model)
		return;

	sc::StructServo servo;
	servo.angle = ui->dsb_gpio_angle->value();
	servo.freq_meandr = ui->dsb_gpio_freq->value();
	servo.timework_ms = ui->dsb_gpio_delay->value();
	servo.pin = ui->sb_gpio_pin->value();
	m_model->send_servo(servo);
}
