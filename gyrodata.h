#ifndef GYRODATA_H
#define GYRODATA_H

#include "virtglobject.h"

#include <QVector>

class GyroData : public VirtGLObject
{
	Q_OBJECT
public:
	enum{
		GYRODATA  = TYPE_VGL + 2
	};

	explicit GyroData(QObject *parent = 0);

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

signals:

public slots:


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

	QVector3D m_center_accel;
};

#endif // GYRODATA_H
