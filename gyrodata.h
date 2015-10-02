#ifndef GYRODATA_H
#define GYRODATA_H

#include "virtglobject.h"

#include <QVector>
#include <QTimer>
#include <QHostAddress>
#include <QTime>
#include <QElapsedTimer>

#include <QQuaternion>

#include "struct_controls.h"
#include "simplekalmanfilter.h"
#include "calibrateaccelerometer.h"

class QUdpSocket;

const int max_count_telemetry = 1000;
const int max_delay_for_data = 200;

/**
 * @brief The GyroData class
 */
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
	QHostAddress addr() const;
	ushort port() const;
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
	 * @brief count_vec_for_calc
	 * @return
	 */
	int count_gyro_offset_data() const;
	/**
	 * @brief reset
	 * reset the initial state
	 */
	void reset();
	/**
	 * @brief set_init_position
	 * reset rotation and moving for the arrows
	 */
	void set_init_position();
	/**
	 * @brief calibrate
	 */
	bool calibrate();

	const CalibrateAccelerometer &calibrateAccelerometer() const;

	bool is_draw_mean_sphere() const;
	void set_draw_mean_sphere(bool value);
	void reset_mean_sphere();

	const StructMeanSphere& mean_sphere() const { return m_sphere; }

	bool is_show_calibrated_data() const { return m_showing_downloaded_data; }
	void set_show_calibrated_data(bool value);

	bool is_write_data() const { return m_write_data; }
	void set_write_data(bool value);
	int count_write_data() const { return m_writed_telemetries.size(); }

signals:
	void get_data(const QString& name, const Vertex3i);
	void get_data(const QString& name, double value);
	void add_to_log(const QString& text);

public slots:
	void on_timeout();
	void on_timeout_playing();
	void on_timeout_calibrate();
	void on_readyRead();

protected:
	void tryParseData(const QByteArray& data);
	void analyze_telemetry(StructTelemetry& st);

	// VirtGLObject interface
public:
	virtual void init();
	virtual void draw();
	virtual void tick();
	virtual QVector3D position() const;

private:
	QString m_fileName;
	QVector< StructTelemetry > m_downloaded_telemetries;
	QUdpSocket *m_socket;
	double m_divider_gyro;
	double m_divider_accel;
	long long m_index;

	bool m_showing_downloaded_data;
	bool m_is_play;
	int m_current_playing_pos;

	double m_percent_downloaded_data;

	QTimer m_timer;
	QTimer m_timer_playing;
	QTimer m_timer_calibrate;

	QVector <StructTelemetry > m_writed_telemetries;
	QVector< StructTelemetry > m_telemetries;
	QTime m_time_waiting_telemetry;
	QElapsedTimer m_tick_telemetry;
	double m_part_of_time;
	long long m_past_tick;
	long long m_first_tick;
	bool m_write_data;

	Vertex3d m_offset_gyro;
	bool m_is_calc_offset_gyro;
	bool m_is_calculated;
	int m_count_gyro_offset_data;
	Vertex3d m_tmp_axes;
	double m_tmp_angle;
	Vertex3d m_mean_accel;
	Vertex3d m_tmp_accel;

	QQuaternion m_rotate_quaternion;
	Vertex3d m_rotate_pos;
	Vertex3d m_translate_pos;
	Vertex3d m_translate_speed;
	Vertex3d m_mean_Gaccel;
	double m_len_Gaccel;
	Vertex3i m_past_accel;

	QHostAddress m_addr;
	ushort m_port;

	SimpleKalmanFilter m_kalman[6];

	StructMeanSphere m_sphere;
	CalibrateAccelerometer m_calibrate;
	bool m_is_draw_mean_sphere;
	QVector< Vertex3f > m_vecs_sphere;
	QVector< Vertex3i > m_inds_sphere;
	bool m_show_calibrated_data;

	QVector < Vertex3f > m_trajectory;

	void init_sphere();

	void calc_offsets(Vertex3i &gyro, Vertex3i &accel);
	void clear_data();
	void load_from_xml();
	void save_to_xml();
	void load_calibrate();
	void save_calibrate();

	void draw_text(const Vertex3d& v, QString text);
	void draw_sphere();
};

#endif // GYRODATA_H
