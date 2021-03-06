#ifndef GYRODATAWIDGET_H
#define GYRODATAWIDGET_H

#include <QWidget>
#include <QTimer>

#include "gyrodata.h"

namespace Ui {
class GyroDataWidget;
}

class GyroDataWidget : public QWidget
{
	Q_OBJECT

public:
	explicit GyroDataWidget(QWidget *parent = 0);

	~GyroDataWidget();
	/**
	 * @brief model
	 * @return
	 */
	GyroData* model() { return m_model; }
	void set_model(GyroData* model);
	/**
	 * @brief is_enable
	 * @return
	 */
	bool is_enable() { return m_model? m_model->is_enable() : false; }
	void set_enable(bool value) { if(m_model) m_model->set_enable(value); }
	/**
	 * @brief type
	 * @return
	 */
	int type() { return m_model? m_model->type() : 0; }

public slots:
	void _on_timeout_cfg();
	void _on_timeout_tmcalib();

	void on_pushButton_5_clicked();

	void on_pushButton_7_clicked();

	void on_pushButton_8_clicked();

	void on_dsb_div_gyro_valueChanged(double arg1);

	void on_dsb_accel_data_valueChanged(double arg1);

	void on_hs_set_end_position_sliderMoved(int position);

	void on_cb_show_loaded_clicked(bool checked);

	void on_dsb_frequency_playing_valueChanged(double arg1);

	void on_hs_playing_data_valueChanged(int value);

	void on_pb_play_clicked(bool checked);

	void on_pb_stop_clicked();

	void on_pushButton_6_clicked(bool checked);

	void on_pb_reset_clicked();

	void on_pushButton_9_clicked();

	void on_pushButton_10_clicked(bool checked);

	void on_chb_calibrate_sphere_clicked(bool checked);

	void on_chb_calibratd_data_clicked(bool checked);

	void on_pushButton_11_clicked(bool checked);

signals:
	void set_status_bar_text(const QString &text);

private slots:
	void on_pb_save_calibration_clicked();

	void on_pushButton_clicked();

	void on_pb_zero_pos_clicked();

	void on_pb_ninety__clicked();

	void on_pb_invert_pos_clicked();

	void on_pb_add_to_pool_clicked(bool checked);

	void on_pb_cancel_clicked();

	void on_pb_load_calibration_clicked();

	void on_chb_recorded_data_clicked(bool checked);

	void on_pb_write_log_clicked();

	void on_pb_calibrate_compass_clicked();

	void on_pb_reset_compass_vcalibrate_clicked();

	void on_le_ip_gyro_data_returnPressed();

	void on_sb_gyro_data_valueChanged(int arg1);

	void on_pb_gpio_send_clicked();

private:
	Ui::GyroDataWidget *ui;
	QTimer m_timer_cfg;
	QTimer m_tmcalib;

	GyroData* m_model;

	void init_model();
};

#endif // GYRODATAWIDGET_H
