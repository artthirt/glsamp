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

	double divider_gyro() const;
	void set_divider_gyro(double val);
	double divider_accel() const;
	void set_divider_accel(double val);

	int shift_gyro() const;
	void set_shift_gyro(int val);
	int shift_accel() const;
	void set_shift_accel(int val);

	/**
	 * @brief set_end_position_loaded_data
	 * @param value - percent for drawing data
	 */
	void set_end_pos_downloaded_data(double value);
	double end_pos_downloaded_data() const;

	bool showing_downloaded_data() const;
	void set_showing_downloaded_data(bool value);

	bool is_play() const;
	void play();
	void stop();
	double percent_position() const;

signals:
	void get_data(const QString& name, const Vertex3i);

public slots:
	void on_timeout();
	void on_timeout_playing();
	void on_readyRead();

protected:
	void tryParseData(const QByteArray& data);

	// VirtGLObject interface
public:
	virtual void init();
	virtual void draw();
	virtual void tick();
	virtual QVector3D position() const;

private:
	QString m_fileName;
	QVector< Vertex3i > m_gyro_data;
	QVector< Vertex3i > m_accel_data;
	QVector< double > m_temp_data;
	QUdpSocket *m_socket;
	double m_divider_gyro;
	double m_divider_accel;
	int m_shift_gyro;
	int m_shift_accel;

	bool m_showing_downloaded_data;
	bool m_is_play;
	int m_current_playing_pos;

	double m_percent_downloaded_data;

	QTimer m_timer;
	QTimer m_timer_playing;

	QVector< StructTelemetry > m_telemtries;
	QTime m_time_waiting_telemetry;

	QVector3D m_center_accel;

	void load_from_xml();
	void save_to_xml();
};

#endif // GYRODATA_H
