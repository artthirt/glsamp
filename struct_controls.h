#ifndef STRUCT_CONTROLS_H
#define STRUCT_CONTROLS_H

#define FOREACH(index, cnt, expression) for(int index = 0; index < cnt; index++){ \
	expression; \
	}

template< typename T >
struct Vertex3_{
	Vertex3_(){
		FOREACH(i, 3, data[i] = 0);
	}
	Vertex3_(T x, T y, T z){
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}
	Vertex3_(const Vertex3_& v){
		FOREACH(i, 3, data[i] = v.data[i];);
	}
	template< typename P >
	Vertex3_(const Vertex3_<P> &v)
	{
		FOREACH(i, 3, data[i] = static_cast< T > (v.data[i]));
	}

	T x() const { return data[0]; }
	T y() const { return data[1]; }
	T z() const { return data[2]; }
	void setX(T value) { data[0] = value; }
	void setY(T value) { data[1] = value; }
	void setZ(T value) { data[2] = value; }

	Vertex3_& operator* (T value){
		FOREACH(i, 3, data[i] *= value);
		return *this;
	}
	Vertex3_& operator*= (T value){
		FOREACH(i, 3, data[i] *= value);
		return *this;
	}
	Vertex3_& operator+= (const Vertex3_& v){
		FOREACH(i, 3, data[i] += v.data[i]);
		return *this;
	}
	Vertex3_& operator-= (const Vertex3_& v){
		FOREACH(i, 3, data[i] -= v.data[i]);
		return *this;
	}

	T data[3];
};

typedef Vertex3_< float > Vertex3f;
typedef Vertex3_< int > Vertex3i;

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

	Vertex3i gyro;

	Vertex3i accel;
};

#endif // STRUCT_CONTROLS_H
