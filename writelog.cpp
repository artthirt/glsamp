#include "writelog.h"

#include <QTextStream>
#include <QDateTime>
#include <QDir>

LogFile::LogFile()
{
	max_line_in_buffer = 1000;
}

LogFile::LogFile(const LogFile &log)
{
	fileName = log.fileName;
	name = log.name;
	logFile.setFileName(log.logFile.fileName());
	max_line_in_buffer = log.max_line_in_buffer;
	data = log.data;
}

LogFile::~LogFile()
{
	logFile.close();
}

LogFile &LogFile::operator=(const LogFile &log)
{
	fileName = log.fileName;
	name = log.name;
	logFile.setFileName(log.logFile.fileName());
	max_line_in_buffer = log.max_line_in_buffer;
	data = log.data;

	return *this;
}

void LogFile::openFile(const QString &name)
{
	if(name.isEmpty())
		return;

	QDir dir(QDir::homePath());

	QString log_path = QDir::homePath() + log_dir;

	if(!dir.exists(log_path)){
		dir.mkdir(log_path);
	}

	QString fn = log_path + name + "_" + QDateTime::currentDateTime().toString() + ".csv";

	if(QFile::exists(fn)){
		QFile::remove(fn);
	}
	logFile.setFileName(fn);
	fileName = fn;

	this->name = name;

	logFile.open(QIODevice::WriteOnly);
}

void LogFile::write_data()
{
	if(!logFile.isOpen())
		return;

	while(data.size()){
		m_mutex.lock();
		QString dt = data.front() + "\n";
		qint64 res = logFile.write(dt.toUtf8());
		data.pop_front();
		m_mutex.unlock();
	}
	logFile.flush();
}

void LogFile::push_data(const QString& name, const QString &str)
{
	if(name.isEmpty())
		return;

	while(data.size() >= max_line_in_buffer){
		m_mutex.lock();
		data.pop_front();
		m_mutex.unlock();
	}

	if(!logFile.isOpen()){
		openFile(name);
	}
	m_mutex.lock();
	data.push_back(str);
	m_mutex.unlock();
}

void LogFile::newLog()
{
	if(name.isEmpty())
		return;

	data.clear();

	logFile.close();
	openFile(name);
}

void LogFile::close()
{
	data.clear();

	logFile.close();
}

////////////////////////////////////////

WriteLog *WriteLog::m_instance = 0;

WriteLog::WriteLog(QObject *parent):
	QThread(parent)
{

}

WriteLog::~WriteLog()
{
	quit();
	wait();
}

WriteLog *WriteLog::instance()
{
	if(!m_instance){
		m_instance = new WriteLog;
		m_instance->moveToThread(m_instance);
		m_instance->start();
	}
	return m_instance;
}

void WriteLog::clearLogs()
{
	for(QMap< QString, LogFile >::iterator it = m_logFiles.begin(); it != m_logFiles.end(); it++){
		it.value().close();
	}
}

void WriteLog::newLogs()
{
	for(QMap< QString, LogFile >::iterator it = m_logFiles.begin(); it != m_logFiles.end(); it++){
		it.value().newLog();
	}
}

void WriteLog::createLog(const QString &name)
{
	m_logFiles[name].openFile(name);
}

void WriteLog::add_data(const QString &name, const QString &data)
{
	m_logFiles[name].push_data(name, data);
}

void WriteLog::add_data(const QString &name, const StructTelemetry &data)
{
	QString str;

#define ADDVAL(val) str += QString::number(val) + ";"

	ADDVAL(data.bank);
	ADDVAL(data.course);
	ADDVAL(data.tangaj);
	ADDVAL(data.height);
	ADDVAL(data.temp);

	ADDVAL(data.accel.x());
	ADDVAL(data.accel.y());
	ADDVAL(data.accel.z());

	ADDVAL(data.gyro.x());
	ADDVAL(data.gyro.y());
	ADDVAL(data.gyro.z());

	m_logFiles[name].push_data(name, str);
}

void WriteLog::on_timeout()
{
	for(QMap< QString, LogFile >::iterator it = m_logFiles.begin(); it != m_logFiles.end(); it++){
		it.value().write_data();
	}
}

void WriteLog::run()
{
	while(1){
		on_timeout();

		QThread::msleep(300);
	}
}
