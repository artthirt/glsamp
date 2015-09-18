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

	void on_pushButton_6_clicked();

protected:
	void init_list_objects();

private:
	Ui::MainWindow *ui;

	QTimer m_timer;
	QTimer m_timer_cfg;

	QuadModel m_model;
	GyroData m_gyroData;
};

#endif // MAINWINDOW_H
