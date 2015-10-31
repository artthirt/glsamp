#ifndef SENSORSWORK_H
#define SENSORSWORK_H

#include <QThread>
#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QTime>

#include "vector3_.h"
#include "matrix3.h"
#include "quaternions.h"
#include "struct_controls.h"

#include <QElapsedTimer>

#include "spheregl.h"
#include "simplekalmanfilter.h"
#include "calibrateaccelerometer.h"

class QUdpSocket;

const int max_count_telemetry = 1000;
const int max_delay_for_data = 200;

class SensorsWork : public QThread
{
	Q_OBJECT
public:

	enum TypeOfCalibrate{
		NONE,
		Accelerometer,
		Compass
	};

	SensorsWork(QObject* parent = 0);
	~SensorsWork();

	QHostAddress addr() const;
	ushort port() const;

	void send_start();
	void send_stop();
	void set_address(const QHostAddress& address, ushort port);

	void set_init_position();
	/**
	 * @brief is_available_telemetry
	 * available telemetry from device
	 * @return
	 */
	bool is_available_telemetry() const;

	int count_gyro_offset_data() const;

	const StructMeanSphere& mean_sphere() const { return m_sphere; }
	const StructMeanSphere& mean_sphere_compass() const { return m_sphere_compass; }
	/**
	 * @brief calibrate
	 */
	bool calibrate_accelerometer(const QVector< vector3_::Vector3d > &data);
	bool calibrate_compass(const QVector< vector3_::Vector3d > &data);

	void reset_calibration_compass();
	void reset_mean_sphere();

	const CalibrateAccelerometer &calibrate_thread() const;
	TypeOfCalibrate typeOfCalibrate() const;

	/// \brief save & load calibrated data
	void save_calibrate();
	void load_calibrate();

	enum POS{
		POS_0 = 0,
		POS_90,
		POS_180
	};
	bool is_exists_value(POS pos) const;
	void set_start_calc_pos(POS pos);
	void set_calccount_pos(int cnt) { m_calccount = cnt; }
	int calccount_pos() const { return m_calccount; }

	bool is_calculated() const { return m_is_calculated; }

	void set_position();
	void start_calc_offset_gyro();
	void stop_calc_offset_gyro();
	void calc_correction();

public:
	/**
	 * @brief analyze_telemetry
	 * analyze telemetry and apply filters for data
	 * @param st_in
	 * @return telemetry with filters
	 */
	sc::StructTelemetry analyze_telemetry(const sc::StructTelemetry& st_in);

	vector3_::Vector3d tmp_accel;
	vector3_::Vector3d meanGaccel;
	vector3_::Vector3d mean_accel;
	quaternions::Quaternion accel_quat;
	quaternions::Quaternion rotate_quaternion;
	QVector< sc::StructTelemetry > telemetries;

signals:
	void get_data(const QString& name, const vector3_::Vector3i);
	void get_data(const QString& name, double value);
	void bind_address();
	void send_to_socket(const QByteArray& data);
	void add_to_log(const QString& text);
	void set_text(const QString& key, const QString text);
	void stop_calibration();
	void fill_data_for_calibration(const sc::StructTelemetry& st);
	void start_calibration_watcher();

protected:
	virtual void run();
	void tryParseData(const QByteArray& data);

public slots:
	void _on_readyRead();
	void _on_timeout_calibrate();
	void _on_timeout();
	void _on_bind_address();
	void _on_send_to_socket(const QByteArray& data);
	void _on_start_calibration_watcher();

private:
	QUdpSocket *m_socket;
	ushort m_receiver_port;
	QMap< POS, vector3_::Vector3d > m_pos_values;
	POS m_curcalc_pos;
	int m_calcid;
	int m_calccount;
	bool m_is_calc_pos;
	qint64 m_index;

	bool m_is_calc_offset_gyro;
	int m_count_gyro_offset_data;
	bool m_is_calculated;

	QTimer *m_timer;
	QTimer *m_timer_calibrate;

	QTime m_time_waiting_telemetry;
	QElapsedTimer m_tick_telemetry;
	double m_part_of_time;
	long long m_past_tick;
	long long m_first_tick;

	vector3_::Vector3d m_offset_gyro;
	vector3_::Vector3d m_tmp_axis;
	double m_tmp_angle;
	vector3_::Vector3d m_prev_accel;

	quaternions::Quaternion m_correct_quaternion;
	matrix::Matrix3d m_corr_matrix;
	vector3_::Vector3d m_rotate_pos;
	vector3_::Vector3d m_translate_pos;
	vector3_::Vector3d m_translate_speed;
	double m_len_Gaccel;
	vector3_::Vector3i m_past_accel;
	double m_threshold_accel;
	bool m_is_start_correction;

	QHostAddress m_addr;
	ushort m_port;

	SimpleKalmanFilter m_kalman[6];

	StructMeanSphere m_sphere;
	StructMeanSphere m_sphere_compass;
	CalibrateAccelerometer m_calibrate;
	TypeOfCalibrate m_typeOfCalibrate;

	double m_max_threshold_angle;
	double m_min_threshold_angle;
	double m_multiply_correction;
	double m_coeff_deltaAngle;

	void calccount(const sc::StructTelemetry& st);
	void calc_offsets(const vector3_::Vector3i &gyro, const vector3_::Vector3i &accel);
	void clear_data();
	/**
	 * @brief simple_kalman_filter
	 * obtain values for gyroscope and accelerometer and apply the  kalman filter for it
	 * @param st
	 * @param st_out
	 */
	void simple_kalman_filter(const sc::StructTelemetry& st, sc::StructTelemetry& st_out);
	void correct_error_gyroscope();
};

#endif // SENSORSWORK_H
