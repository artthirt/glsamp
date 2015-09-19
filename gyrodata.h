#ifndef GYRODATA_H
#define GYRODATA_H

#include "virtglobject.h"

#include <QVector>
#include <QTimer>
#include <QHostAddress>
#include <QTime>

#include "struct_controls.h"

class QUdpSocket;

const int max_count_telemetry = 1000;
const int max_delay_for_data = 200;

class GyroData : public VirtGLObject
{
	Q_OBJECT
public:
	enum{
		GYRODATA  = TYPE_VGL + 2
	};

	explicit GyroData(QObject *parent = 0);
	virtual ~GyroData();

	/**
	 * @brief fileName
	 * @return
	 */
	QString fileName() const;
	/**
	 * @brief openFile
	 * @param fileName
	 */
	void openFile(const QString fileName);
	/**
	 * @brief recalc_accel
	 * @param threshold
	 */
	void recalc_accel(double threshold, double threshold_deriv = 0.01);

	void send_start_to_net(const QHostAddress& host, ushort port);
	void send_stop_to_net(const QHostAddress& host, ushort port);
	bool is_available_telemetry() const;

signals:

public slots:
	void on_timeout();
	void on_readyRead();

	// VirtGLObject interface
public:
	virtual void init();
	virtual void draw();
	virtual void tick();
	virtual QVector3D position() const;

private:
	QString m_fileName;
	QVector< QVector3D > m_gyro_data;
	QVector< QVector3D > m_accel_data;
	QVector< double > m_temp_data;
	QUdpSocket *m_socket;

	QTimer m_timer;

	QVector< StructTelemetry > m_telemtries;
	QTime m_time_waiting_telemetry;

	QVector3D m_center_accel;

	void tryParseData(const QByteArray& data);
};

#endif // GYRODATA_H
