#ifndef STRUCT_CONTROLS_H
#define STRUCT_CONTROLS_H

#include <vector>
#include <QDataStream>

/// \brief FOREACH - cycle for the expression at
/// the specified index and the number of repetitions
#define FOREACH(index, cnt, expression) for(int index = 0; index < cnt; index++){ \
	expression; \
	}

class ExceptionCustom{
public:
	enum{
		UNKNOWN = 0
	};
	std::string message;
	int code;

	ExceptionCustom(std::string msg){
		message = msg;
		code = UNKNOWN;
	}
	ExceptionCustom(int code){
		this->code = code;
	}
};

#ifdef _DEBUG
#define ASSERT_EC(val, msg)	if(!(val)) throw new ExceptionCustom(msg)
#else
#define ASSERT_EC(val, msg)
#endif

template< typename T >
struct Vertex3_{
	enum{
		count = 3
	};
	Vertex3_(){
		FOREACH(i, count, data[i] = 0);
	}
	Vertex3_(T x, T y, T z){
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}
	Vertex3_(const Vertex3_& v){
		FOREACH(i, count, data[i] = v.data[i];);
	}
	template< typename P >
	Vertex3_(const Vertex3_<P> &v)
	{
		FOREACH(i, count, data[i] = static_cast< T > (v.data[i]));
	}

	T x() const { return data[0]; }
	T y() const { return data[1]; }
	T z() const { return data[2]; }
	void setX(T value) { data[0] = value; }
	void setY(T value) { data[1] = value; }
	void setZ(T value) { data[2] = value; }

	Vertex3_& operator* (T value){
		FOREACH(i, count, data[i] *= value);
		return *this;
	}
	Vertex3_& operator*= (T value){
		FOREACH(i, count, data[i] *= value);
		return *this;
	}
	Vertex3_& operator+= (const Vertex3_& v){
		FOREACH(i, count, data[i] += v.data[i]);
		return *this;
	}
	Vertex3_& operator-= (const Vertex3_& v){
		FOREACH(i, count, data[i] -= v.data[i]);
		return *this;
	}
	T& operator[] (int index){
		ASSERT_EC(index >=0 && index < count, "index out of range");
		return data[index];
	}
	T& operator[] (int index) const{
		ASSERT_EC(index >=0 && index < count, "index out of range");
		return data[index];
	}

	T data[count];
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
const int raw_count = 46;

struct StructTelemetry
{
	/**
	 * @brief StructTelemetry
	 */
	StructTelemetry(){
		FOREACH(i, cnt_engines, power[i] = 0;);
		power_on = false;
		height = 0;
		temp = 0;
		course = tangaj = bank = 0;
		fs_sel = 0;
		afs_sel = 0;
		FOREACH(i, raw_count, raw[i] = 0);
	}
	/**
	 * @brief StructTelemetry
	 * @param st
	 */
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
		afs_sel = st.afs_sel;
		fs_sel = st.fs_sel;
		std::copy(st.raw, st.raw + raw_count, raw);
	}
	/**
	 * @brief write_to
	 * serialize to byte array
	 * @param stream
	 */
	void write_to(QDataStream& stream){
		stream.setByteOrder(QDataStream::BigEndian);
		stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
		stream.setVersion(QDataStream::Qt_4_8);

		stream << power_on;
		FOREACH(i, cnt_engines, stream << power[i]);
		stream << tangaj;
		stream << bank;
		stream << course;
		stream << temp;
		stream << height;
		FOREACH(i, Vertex3i::count, stream << gyro[i]);
		FOREACH(i, Vertex3i::count, stream << accel[i]);
		stream << afs_sel;
		stream << fs_sel;
		stream.writeRawData(reinterpret_cast< char* >(raw), raw_count);
	}
	/**
	 * @brief read_from
	 * deserialize byte array
	 * @param stream
	 */
	void read_from(QDataStream& stream){
		stream.setByteOrder(QDataStream::BigEndian);
		stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
		stream.setVersion(QDataStream::Qt_4_8);

		stream >> power_on;
		FOREACH(i, cnt_engines, stream >> power[i]);
		stream >> tangaj;
		stream >> bank;
		stream >> course;
		stream >> temp;
		stream >> height;
		FOREACH(i, Vertex3i::count, stream >> gyro[i]);
		FOREACH(i, Vertex3i::count, stream >> accel[i]);
		stream >> afs_sel;
		stream >> fs_sel;
		stream.readRawData(reinterpret_cast< char* >(raw), raw_count);
	}

	Vertex3f angular_speed(){
		float factor = 1.0;
		switch (fs_sel) {
			case 0:
			default:
				factor = 250.0f / 32768.0f;
				break;
			case 1:
				factor = 500.0f / 32768.0f;
				break;
			case 2:
				factor = 1000.0f / 32768.0f;
				break;
			case 3:
				factor = 2000.0f / 32768.0f;
				break;
		}
		Vertex3f res = Vertex3f(gyro);
		res *= factor;
		return res;
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

	unsigned char afs_sel;				/// value of accelerometer mode
	unsigned char fs_sel;				/// value of gyroscope mode
	unsigned char raw[raw_count];		/// raw data from 0x0d to 0x3a address from mpu6050
};

#endif // STRUCT_CONTROLS_H
