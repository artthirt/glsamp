#ifndef GYRODATA_H
#define GYRODATA_H

#include "virtglobject.h"

#include <QVector>
#include <QTimer>
#include <QHostAddress>
#include <QTime>
#include <QElapsedTimer>
#include <QMap>
#include <QColor>

#include "struct_controls.h"
#include "matrix3.h"

#include "spheregl.h"
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

	/// \brief draw calculation sphere for finded clibration values
	bool is_draw_mean_sphere() const;
	void set_draw_mean_sphere(bool value);
	void reset_mean_sphere();

	const StructMeanSphere& mean_sphere() const { return m_sphere; }

	/// \brief show datat in view with calibration values
	bool is_show_calibrated_data() const { return m_show_calibrated_data; }
	void set_show_calibrated_data(bool value);

	/// \brief create array in real time for use in calibration
	bool is_write_data() const { return m_write_data; }
	void set_write_data(bool value);
	int count_write_data() const;
	void add_to_pool(bool value);
	void cancel_write();
	bool is_add_to_pool() const { return m_add_to_pool; }
	void log_recorded_data();

	/// \brief save & load calibrated data
	void save_calibrate();
	void load_calibrate();

	void reset_trajectory();

	void set_text(const QString& key, const QString text);
	void set_visible_text(bool value);
	bool is_visible_text() const;

	enum POS{
		POS_0 = 0,
		POS_90,
		POS_180
	};
	bool is_exists_value(POS pos) const;
	void set_start_calc_pos(POS pos);
	void set_calccount_pos(int cnt) { m_calccount = cnt; }
	int calccount_pos() const { return m_calccount; }

	void show_recorded_data(bool value);
	bool is_show_recorded_data() const {return m_show_recorded_data;}

private:
	QMap< POS, sc::Vector3d > m_pos_values;
	POS m_curcalc_pos;
	int m_calcid;
	int m_calccount;
	bool m_is_calc_pos;
	bool m_show_recorded_data;

	void calccount(const sc::StructTelemetry& st);

signals:
	void get_data(const QString& name, const sc::Vector3i);
	void get_data(const QString& name, double value);
	void add_to_log(const QString& text);

public slots:
	void on_timeout();
	void on_timeout_playing();
	void on_timeout_calibrate();
	void on_readyRead();
	void on_calibrate_log(const QString& data);

protected:
	void tryParseData(const QByteArray& data);
	void analyze_telemetry(sc::StructTelemetry& st);

	// VirtGLObject interface
public:
	virtual void init();
	virtual void draw();
	virtual void tick();
	virtual QVector3D position() const;

private:
	QString m_fileName;
	QVector< sc::StructTelemetry > m_downloaded_telemetries;
	QUdpSocket *m_socket;
	double m_divider_accel;
	double m_divider_gyro;
	double m_percent_downloaded_data;
	long long m_index;

	QMap< QString, QString > m_drawing_text;
	bool m_is_visible_text;

	bool m_showing_downloaded_data;
	bool m_is_play;
	int m_current_playing_pos;


	QTimer m_timer;
	QTimer m_timer_playing;
	QTimer m_timer_calibrate;

	bool m_is_calc_offset_gyro;
	int m_count_gyro_offset_data;
	bool m_is_calculated;

	QVector< sc::StructTelemetry > m_writed_telemetries;
	QVector< sc::StructTelemetry > m_pool_writed_telemetries;
	QVector< sc::StructTelemetry > m_telemetries;
	QTime m_time_waiting_telemetry;
	QElapsedTimer m_tick_telemetry;
	double m_part_of_time;
	long long m_past_tick;
	long long m_first_tick;
	bool m_write_data;
	bool m_add_to_pool;

	sc::Vector3d m_offset_gyro;
	sc::Vector3d m_tmp_axis;
	double m_tmp_angle;
	sc::Vector3d m_mean_accel;
	sc::Vector3d m_tmp_accel;
	sc::Vector3d m_prev_accel;

	sc::Quaternion m_rotate_quaternion;
	sc::Quaternion m_correct_quaternion;
	sc::Quaternion m_accel_quat;
	matrix::Matrix3d m_corr_matrix;
	sc::Vector3d m_rotate_pos;
	sc::Vector3d m_translate_pos;
	sc::Vector3d m_translate_speed;
	sc::Vector3d m_mean_Gaccel;
	double m_len_Gaccel;
	sc::Vector3i m_past_accel;
	double m_threshold_accel;

	QHostAddress m_addr;
	ushort m_port;

	SimpleKalmanFilter m_kalman[6];

	SphereGL m_sphereGl;
	StructMeanSphere m_sphere;
	CalibrateAccelerometer m_calibrate;
	bool m_is_draw_mean_sphere;
	bool m_show_calibrated_data;

	QVector < sc::Vector3f > m_trajectory;

	double m_threshold_angle;
	double m_multiply_correction;

	void init_sphere();

	void calc_offsets(sc::Vector3i &gyro, sc::Vector3i &accel);
	void clear_data();
	void load_from_xml();
	void save_to_xml();

	void draw_text(const sc::Vector3d& v, QString text, const QColor &col = Qt::white);
	void draw_sphere();
	void draw_text();
	void draw_recored_data();

	void calc_parameters();
	void calc_correction();
	void calc_calibrations_params(const sc::StructTelemetry& st);
	void simple_kalman_filter(sc::StructTelemetry& st);
	void correct_error_gyroscope();
};

#endif // GYRODATA_H
