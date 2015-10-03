#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "quadmodel.h"
#include "gyrodata.h"

namespace Ui {
class MainWindow;
}

class QListWidgetItem;
class QLabel;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void on_pb_left_front_clicked();

	void on_pb_zero_clicked();

	void on_pb_right_front_clicked();

	void on_pb_left_back_clicked();

	void on_pb_down_clicked();

	void on_pb_up_clicked();

	void on_pb_right_back_clicked();

	void on_timeout();
	void on_timeout_cfg();
	void on_timeout_tmcalib();

	void on_vs_power_valueChanged(int value);

	void on_pb_zero_2_clicked();

	void on_cb_watch_clicked(bool checked);

	void on_cb_watch_gl_clicked(bool checked);

	void on_pushButton_2_clicked();

	void on_pushButton_clicked();

	void on_pushButton_3_clicked();

	void on_pushButton_4_clicked();

	void on_dsb_mean_valueChanged(double arg1);

	void on_dsb_sigma_valueChanged(double arg1);

	void on_lw_objects_itemChanged(QListWidgetItem *item);

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

	void on_chb_draw_lever_clicked(bool checked);

	void on_pushButton_10_clicked(bool checked);

	void on_put_data(const QString &name, const Vector3i &value);
	void on_put_data(const QString &name, double value);

	void on_pb_clear_log_clicked();

	void on_chb_calibrate_sphere_clicked(bool checked);

	void add_to_log(const QString& text);

	void on_chb_calibratd_data_clicked(bool checked);

	void on_pushButton_11_clicked(bool checked);

	void on_pushButton_12_clicked();

protected:
	void init_list_objects();

private:
	Ui::MainWindow *ui;

	QTimer m_timer;
	QTimer m_timer_cfg;
	QTimer m_tmcalib;

	QLabel *m_available_telemetry;

	QuadModel m_model;
	GyroData m_gyroData;

	void load_from_xml();
	void save_to_xml();
};

#endif // MAINWINDOW_H
