#include "dialogabout.h"
#include "ui_dialogabout.h"

#include "global.h"

DialogAbout::DialogAbout(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::DialogAbout)
{
	ui->setupUi(this);

	m_ver_build = QString(VERBUILD).left(6);
	m_version = version_string + "-" + m_ver_build;

	ui->lb_version->setText(m_version);
}

DialogAbout::~DialogAbout()
{
	delete ui;
}
