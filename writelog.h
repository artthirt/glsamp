#ifndef WRITELOG_H
#define WRITELOG_H

#include <QObject>
#include <QThread>
#include <QMap>
#include <QVector>
#include <QFile>
#include <QMutex>

#include <global.h>
#include <struct_controls.h>

//////////////////////////////////////
/// \brief The LogFile struct
/// struct for write data to log file

struct LogFile{
	QString fileName;
	QString name;
	QVector< QString > data;
	QFile logFile;
	int max_line_in_buffer;
	QMutex m_mutex;

	LogFile();
	explicit LogFile(const LogFile& log);
	~LogFile();

	LogFile &operator=( const LogFile& log );

	void openFile(const QString& name);
	void write_data();
	void push_data(const QString& name, const QString& str);
	void newLog();
	void close();
};

////////////////////////////////////
/// \brief The WriteLog class
/// to control class of logs
class WriteLog : public QThread
{
public:
	WriteLog(QObject *parent = 0);
	~WriteLog();

	static WriteLog *instance();

	void clearLogs();
	void newLogs();

	void createLog(const QString& name);
	void add_data(const QString & name, const QString &data);
	void add_data(const QString & name, const sc::StructTelemetry &data);
	void write_data(const QString & name, const QVector< sc::StructTelemetry > &data);
	void closeLog(const QString& name);
signals:

public slots:
	void _on_timeout();

protected:
	virtual void run();

private:
	QMap< QString, LogFile > m_logFiles;

	static WriteLog *m_instance;
};

#endif // WRITELOG_H
