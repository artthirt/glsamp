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

void MainWindow::on_pb_left_front_clicked()
{

}

void MainWindow::on_pb_zero_clicked()
{

}

void MainWindow::on_pb_right_front_clicked()
{

}

void MainWindow::on_pb_left_back_clicked()
{

}

void MainWindow::on_pb_down_clicked()
{
	m_model.add_power(-0.005);
}

void MainWindow::on_pb_up_clicked()
{
	m_model.add_power(0.005);
}

void MainWindow::on_pb_right_back_clicked()
{

}

void MainWindow::on_timeout()
{
	ui->label_alpha->setText(QString::number(m_model.alpha()));
	ui->label_betha->setText(QString::number(m_model.betha()));
	ui->label_gamma->setText(QString::number(m_model.gamma()));
	ui->label_power->setText(QString::number(m_model.power()));
	ui->label_mg->setText(QString::number(m_model.mg()));

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
