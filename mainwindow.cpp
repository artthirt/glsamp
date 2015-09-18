#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include "QListWidgetItem"

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	ui->widget->add_object(&m_model);
	ui->widget->add_object(&m_gyroData);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_timeout()));
	m_timer.start(100);
	connect(&m_timer_cfg, SIGNAL(timeout()), this, SLOT(on_timeout_cfg()));
	m_timer_cfg.start(300);

	init_list_objects();

	ui->lb_filename->setText(m_gyroData.fileName());

	setWindowState( Qt::WindowMaximized );
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
