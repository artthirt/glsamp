#include "gyrodatawidget.h"
#include "ui_gyrodatawidget.h"

#include <QFileDialog>

GyroDataWidget::GyroDataWidget(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::GyroDataWidget)
  , m_model(0)
{
	ui->setupUi(this);

	connect(&m_timer_cfg, SIGNAL(timeout()), this, SLOT(on_timeout_cfg()));
	m_timer_cfg.start(300);

	connect(&m_tmcalib, SIGNAL(timeout()), this, SLOT(on_timeout_tmcalib()));
	m_tmcalib.setInterval(50);

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
}

void GyroDataWidget::on_timeout_cfg()
{
	if(!m_model)
		return;
	if(m_model->is_available_telemetry()){
		emit set_status_bar_text("telemetry received");
	}else{
		emit set_status_bar_text("telemetry not received");
	}

	if(m_model->is_play()){
		ui->hs_playing_data->setValue(m_model->percent_position());
	}
	ui->lb_count_value->setText("count: " + QString::number(m_model->count_gyro_offset_data()));
	ui->lb_write_data->setText("count: " + QString::number(m_model->count_write_data()));

	const QString stsh("background: lightgreen;");
	if(m_model->is_exists_value(GyroData::POS_0)){
		ui->pb_zero_pos->setStyleSheet(stsh);
	}
	if(m_model->is_exists_value(GyroData::POS_90)){
		ui->pb_ninety_->setStyleSheet(stsh);
	}
	if(m_model->is_exists_value(GyroData::POS_180)){
		ui->pb_invert_pos->setStyleSheet(stsh);
	}
}

void GyroDataWidget::on_timeout_tmcalib()
{
	if(!m_model)
		return;
	if(m_model->calibrateAccelerometer().is_done()){
		m_tmcalib.stop();
		ui->widget_pass->setVisible(false);
	}
	ui->pb_calibrate->setValue(m_model->calibrateAccelerometer().pass_part_evaluate() * 100.0);
	ui->lb_pass->setText("pass: " + QString::number(m_model->calibrateAccelerometer().pass()));
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
	m_model->send_start_to_net(QHostAddress(ui->le_ip_gyro_data->text()), ui->sb_gyro_data->value());
}

void GyroDataWidget::on_pushButton_8_clicked()
{
	if(!m_model)
		return;
	m_model->send_stop_to_net(QHostAddress(ui->le_ip_gyro_data->text()), ui->sb_gyro_data->value());
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
		m_model->start_calc_offset_gyro();
		ui->lb_count_value->setStyleSheet("background: lightgreen;");
	}else{
		m_model->stop_calc_offset_gyro();
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
	if(m_model->calibrate()){
		ui->widget_pass->setVisible(true);
		ui->pb_calibrate->setValue(0);
		ui->lb_pass->setText("pass: " + QString::number(m_model->calibrateAccelerometer().pass()));
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
		m_model->save_calibrate();
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
		m_model->set_start_calc_pos(GyroData::POS_0);
	}
}

void GyroDataWidget::on_pb_ninety__clicked()
{
	ui->pb_ninety_->setStyleSheet("");
	if(m_model){
		m_model->set_start_calc_pos(GyroData::POS_90);
	}
}

void GyroDataWidget::on_pb_invert_pos_clicked()
{
	ui->pb_invert_pos->setStyleSheet("");
	if(m_model){
		m_model->set_start_calc_pos(GyroData::POS_180);
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
		m_model->load_calibrate();
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
