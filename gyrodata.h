#ifndef GYRODATA_H
#define GYRODATA_H

#include "virtglobject.h"

#include <QVector>
#include <QTimer>
#include <QHostAddress>
#include <QTime>

#include "struct_controls.h"
#include "simplekalmanfilter.h"

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

	/**
	 * @brief send_start_to_net
	 * signal for start send data from device
	 * @param host
	 * @param port
	 */
	void send_start_to_net(const QHostAddress& host, ushort port);
	/**
	 * @brief send_stop_to_net
	 * signal for sotp send data from device
	 * @param host
	 * @param port
	 */
	void send_stop_to_net(const QHostAddress& host, ushort port);
	/**
	 * @brief is_available_telemetry
	 * available telemetry from device
	 * @return
	 */
	bool is_available_telemetry() const;

	/**
	 * @brief divider_gyro
	 * divider for gyroscope
	 * @return
	 */
	double divider_gyro() const;
	/**
	 * @brief set_divider_gyro
	 * set divider for gyroscope
	 * @param val
	 */
	void set_divider_gyro(double val);
	/**
	 * @brief divider_accel
	 * divider for accelerometer
	 * @return
	 */
	double divider_accel() const;
	/**
	 * @brief set_divider_accel
	 * set divider for accelerometer
	 * @param val
	 */
	void set_divider_accel(double val);

	/**
	 * @brief shift_gyro
	 * count of significant bits of data from gyroscope
	 * @return
	 */
	int shift_gyro() const;
	/**
	 * @brief set_shift_gyro
	 * set count of significant bits of data from gyroscope
	 * @param val
	 */
	void set_shift_gyro(int val);
	/**
	 * @brief shift_accel
	 * count of significant bits of data from accelerometer
	 * @return
	 */
	int shift_accel() const;
	/**
	 * @brief set_shift_accel
	 * set count of significant bits of data from accelerometer
	 * @param val
	 */
	void set_shift_accel(int val);

	/**
	 * @brief set_end_position_loaded_data
	 * @param value - percent for drawing data
	 */
	void set_end_pos_downloaded_data(double value);
	/**
	 * @brief end_pos_downloaded_data
	 * current position representing of downloaded data
	 * @return
	 */
	double end_pos_downloaded_data() const;

	/**
	 * @brief showing_downloaded_data
	 * current state of visible data
	 * @return
	 */
	bool showing_downloaded_data() const;
	/**
	 * @brief set_showing_downloaded_data
	 * use for visible downloaded data
	 * @param value
	 */
	void set_showing_downloaded_data(bool value);

	/**
	 * @brief is_play
	 * current playback state
	 * @return
	 */\
	bool is_play() const;
	/**
	 * @brief play
	 * start playback
	 */
	void play();
	/**
	 * @brief pause
	 * pause for playback
	 */
	void pause();
	/**
	 * @brief stop
	 * stop playback
	 */
	void stop();
	/**
	 * @brief percent_position
	 * position of playback data
	 * @return
	 */
	double percent_position() const;
	/**
	 * @brief set_position_playback
	 * set position in percent for playback
	 * @param position
	 */
	void set_position_playback(double position);

	/**
	 * @brief freq_playing
	 * current frequency of playing
	 * @return
	 */
	double freq_playing() const;
	/**
	 * @brief set_freq_playing
	 * set frequency of playing
	 * @param value
	 */
	void set_freq_playing(double value);
	/**
	 * @brief start_calc_center_gyro
	 */
	void start_calc_offset_gyro();
	/**
	 * @brief stop_calc_center_gyro
	 */
	void stop_calc_offset_gyro();
	/**
	 * @brief reset
	 */
	void reset();

signals:
	void get_data(const QString& name, const Vertex3i);
	void get_data(const QString& name, double value);

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

	Vertex3f m_offset_gyro;
	bool m_is_calc_offset_gyro;
	Vertex3f m_offset_accel;
	int m_count_gyro_offset_data;

	Vertex3f m_rotate_pos;
	QTime m_data_freq_calc;
	qint64 m_count_data_recv;
	double m_data_freq;

	SimpleKalmanFilter m_kalman[6];

	void calc_gyro_offset(Vertex3i &gyro);
	void calc_data_freq();
	void clear_data();
	void load_from_xml();
	void save_to_xml();
};

#endif // GYRODATA_H
