#ifndef STRUCT_CONTROLS_H
#define STRUCT_CONTROLS_H

#include <QVector3D>

#define FOREACH(index, cnt, expression) for(int index = 0; index < cnt; index++){ \
	expression \
	}

struct StructControls
{
	bool power_on;
	float throttle;
	float tangaj;
	float bank;
	float rotation;
};

const int cnt_engines = 4;

struct StructTelemetry
{
	StructTelemetry(){
		FOREACH(i, cnt_engines, power[i] = 0;);
		power_on = false;
		height = 0;
		course = tangaj = bank = 0;
	}
	StructTelemetry(const StructTelemetry& st){
		power_on = st.power_on;
		FOREACH(i, cnt_engines, power[i] = st.power[i];);
		tangaj = st.tangaj;
		bank = st.bank;
		course = st.course;
		gyro = st.gyro;
		accel = st.accel;
		height = st.height;
	}

	bool power_on;

	float power[cnt_engines];

	float tangaj;
	float bank;
	float course;

	QVector3D gyro;

	QVector3D accel;
	float height;
};

#endif // STRUCT_CONTROLS_H
