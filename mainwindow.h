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
	void on_lw_objects_itemChanged(QListWidgetItem *item);

	void _on_put_data(const QString &name, const vector3_::Vector3i &value);
	void _on_put_data(const QString &name, double value);

	void on_pb_clear_log_clicked();

	void add_to_log(const QString& text);

	void on_actionAbout_triggered();

	void _on_status_bar_text(const QString& text);

protected:
	void init_list_objects();

private:
	Ui::MainWindow *ui;

	QLabel *m_available_telemetry;

	void load_from_xml();
	void save_to_xml();
};

#endif // MAINWINDOW_H
