#ifndef STRUCT_CONTROLS_H
#define STRUCT_CONTROLS_H

#define FOREACH(index, cnt, expression) for(int index = 0; index < cnt; index++){ \
	expression; \
	}

struct Vector3f{
	Vector3f(){
		FOREACH(i, 3, data[i] = 0);
	}
	Vector3f(float x, float y, float z){
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}
	Vector3f(const Vector3f& v){
		FOREACH(i, 3, data[i] = v.data[i];);
	}

	float x() const { return data[0]; }
	float y() const { return data[1]; }
	float z() const { return data[2]; }
	void setX(float value) { data[0] = value; }
	void setY(float value) { data[1] = value; }
	void setZ(float value) { data[2] = value; }

	Vector3f& operator* (float value){
		FOREACH(i, 3, data[i] *= value);
		return *this;
	}

	float data[3];
};

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
		temp = 0;
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
		temp = st.temp;
	}

	bool power_on;

	float power[cnt_engines];

	float tangaj;
	float bank;
	float course;
	float temp;
	float height;

	Vector3f gyro;

	Vector3f accel;
};

#endif // STRUCT_CONTROLS_H
