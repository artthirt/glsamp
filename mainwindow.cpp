#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	ui->widget->add_object(&m_model);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_timeout()));
	m_timer.start(100);
	connect(&m_timer_cfg, SIGNAL(timeout()), this, SLOT(on_timeout_cfg()));
	m_timer_cfg.start(300);
}

MainWindow::~MainWindow()
{
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
	ui->label_alpha->setText(QString::number(m_model.alpha()));
	ui->label_betha->setText(QString::number(m_model.betha()));
	ui->label_gamma->setText(QString::number(m_model.gamma()));
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
