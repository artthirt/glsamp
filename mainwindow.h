#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "quadmodel.h"

namespace Ui {
class MainWindow;
}

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

private:
	Ui::MainWindow *ui;

	QTimer m_timer;
	QTimer m_timer_cfg;

	QuadModel m_model;
};

#endif // MAINWINDOW_H
