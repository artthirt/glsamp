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

#include "sensorswork.h"

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
	void set_address(const QHostAddress& host, ushort port);
	/**
	 * @brief send_start_to_net
	 * signal for start send data from device
	 * @param host
	 * @param port
	 */
	void send_start_to_net();
	/**
	 * @brief send_stop_to_net
	 * signal for sotp send data from device
	 * @param host
	 * @param port
	 */
	void send_stop_to_net();

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

	/// \brief create array in real time for use in calibration
	bool is_write_data() const { return m_write_data; }
	void set_write_data(bool value);
	int count_write_data() const;
	void add_to_pool(bool value);
	void cancel_write();
	bool is_add_to_pool() const { return m_add_to_pool; }

	/// \brief draw calculation sphere for found calibration values
	bool is_draw_mean_sphere() const;
	void set_draw_mean_sphere(bool value);

	/// \brief show data in view with calibration values
	bool is_show_calibrated_data() const { return m_show_calibrated_data; }
	void set_show_calibrated_data(bool value);

	void reset_trajectory();

	void set_text(const QString& key, const QString text);
	void set_visible_text(bool value);
	bool is_visible_text() const;

	void show_recorded_data(bool value);
	bool is_show_recorded_data() const {return m_show_recorded_data;}

	bool calibrate_compass();
	bool calibrate_accelerometer();

	void log_recorded_data();

	SensorsWork *sensorsWork();

private:
	bool m_show_recorded_data;

signals:
	void get_data(const QString& name, const vector3_::Vector3i);
	void get_data(const QString& name, double value);
	void add_to_log(const QString& text);

public slots:
	void on_timeout();
	void on_timeout_playing();
	void on_timeout_calibrate();
	void on_calibrate_log(const QString& data);

protected:

	// VirtGLObject interface
public:
	virtual void init();
	virtual void draw();
	virtual void tick();
	virtual QVector3D position() const;

private:
	SensorsWork* m_sensorsWork;

	QString m_fileName;
	QVector< sc::StructTelemetry > m_downloaded_telemetries;
	double m_divider_accel;
	double m_divider_gyro;
	double m_percent_downloaded_data;
	long long m_index;

	QMap< QString, QString > m_drawing_text;
	bool m_is_visible_text;

	bool m_showing_downloaded_data;
	bool m_is_play;
	int m_current_playing_pos;

	QTimer m_timer_playing;

	QVector< sc::StructTelemetry > m_writed_telemetries;
	QVector< sc::StructTelemetry > m_pool_writed_telemetries;
	bool m_write_data;
	bool m_add_to_pool;

	bool m_is_draw_mean_sphere;
	bool m_show_calibrated_data;

	QVector < vector3_::Vector3d > m_trajectory;

	SphereGL m_sphereGl;

	double m_max_threshold_angle;
	double m_min_threshold_angle;
	double m_multiply_correction;

	void init_sphere();

	void clear_data();
	void load_from_xml();
	void save_to_xml();

	void draw_text(const vector3_::Vector3d& v, const QString& text, const QColor &col = Qt::white);
	void draw_sphere();
	void draw_text();
	void draw_recored_data();

	void fill_data_for_calibration(const sc::StructTelemetry& st);
};

#endif // GYRODATA_H
