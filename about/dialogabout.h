#ifndef DIALOGABOUT_H
#define DIALOGABOUT_H

#include <QDialog>

namespace Ui {
class DialogAbout;
}

class DialogAbout : public QDialog
{
	Q_OBJECT

public:
	explicit DialogAbout(QWidget *parent = 0);
	~DialogAbout();

	QString version() const { return m_version; }

private:
	Ui::DialogAbout *ui;

	QString m_ver_build;
	QString m_version;
};

#endif // DIALOGABOUT_H
