#include <asio.hpp>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
using namespace std;

#ifndef tsns
#define tsns chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now()).time_since_epoch().count()
#define tsus chrono::time_point_cast<chrono::microseconds>(chrono::system_clock::now()).time_since_epoch().count()
#define tsms chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count()
#define tsse chrono::time_point_cast<chrono::seconds>(chrono::system_clock::now()).time_since_epoch().count()
#define tsmi chrono::time_point_cast<chrono::minutes>(chrono::system_clock::now()).time_since_epoch().count()
#define tsho chrono::time_point_cast<chrono::hours>(chrono::system_clock::now()).time_since_epoch().count()
#endif

template<typename Object, int nbuf = 9, int nchn = 9, int ndim = 6> class CirArr
{
public:
	static CirArr& GetMe(int dim = 0) { static CirArr us[ndim]; return us[dim]; }

protected:
	bool state = false;
	Object objects[nbuf];
	int64 readPos[nchn] = { 0 };
	int64 writePos = 0;

public:
	bool getState() { return state; }
	int getBuf() { return nbuf; }
	int getChn() { return nchn; }
	int getDim() { return ndim; }

public:
	virtual bool init()
	{
		if (state == true) { spdlog::info("zero operation"); return true; }
		memset(objects, 0, sizeof(objects));
		memset(readPos, 0, sizeof(readPos));
		memset(&writePos, 0, sizeof(writePos));
		state = true;
		return true;
	}
	virtual bool deinit()
	{
		if (state == false) { spdlog::info("zero operation"); return true; }
		memset(objects, 0, sizeof(objects));
		memset(readPos, 0, sizeof(readPos));
		memset(&writePos, 0, sizeof(writePos));
		state = false;
		return true;
	}

public:
	bool getLatest(Object** object, int chnId, int msTimeout = 1000, int msSleep = 2)
	{
		if (state != true) { spdlog::error("wrong state"); return false; }
		for (int64 t0 = tsms; tsms - t0 < msTimeout;)
		{
			int64 availablePos = writePos;
			if (availablePos > readPos[chnId])
			{
				int64 relativePos = availablePos % nbuf;
				*object = objects + relativePos;
				readPos[chnId] = availablePos;
				return true;
			}
			this_thread::sleep_for(chrono::milliseconds(msSleep));
		}
		return false;
	}
	int64 lockWritten(Object** object)
	{
		if (state != true) { spdlog::error("wrong state"); return -1; }
		int64 absolutePos = writePos;
		int64 relativePos = ++absolutePos % nbuf;
		*object = objects + relativePos;
		return absolutePos;
	}
	int64 unlockWritten(int64 absolutePos)
	{
		if (state != true) { spdlog::error("wrong state"); return -1; }
		return (writePos = absolutePos);
	}
};

class XQ4IO
{
public:

public:
	struct XQFrame
	{
		int status;//小车状态: 0未初始化, 1正常, -1表示异常
		float power;//电源电压: 9~13V
		float theta;//方位角:0~360Deg
		int encoder_ppr;//车轮一转对应的编码器个数
		int encoder_delta_r;//右轮编码器增量: 单位个
		int encoder_delta_l;//左轮编码器增量: 单位个
		int encoder_delta_car;//两车轮中心位移: 单位个
		int omga_r;//右轮转速: 个每秒
		int omga_l;//左轮转速: 个每秒
		float distance1;//第一个超声模块距离值: 单位cm
		float distance2;//第二个超声模块距离值: 单位cm
		float distance3;//第三个超声模块距离值: 单位cm
		float distance4;//第四个超声模块距离值: 单位cm
		float imudata[9];//mpu9250 9轴数据
		uint timestamp;//时间戳
		string print(string savePath = "")
		{
			string str;
			str += fmt::format("status: {}\n", status);
			str += fmt::format("power: {}\n", power);
			str += fmt::format("theta: {}\n", theta);
			str += fmt::format("encoder_ppr: {}\n", encoder_ppr);
			str += fmt::format("encoder_delta_r: {}\n", encoder_delta_r);
			str += fmt::format("encoder_delta_l: {}\n", encoder_delta_l);
			str += fmt::format("encoder_delta_car: {}\n", encoder_delta_car);
			str += fmt::format("omga_r: {}\n", omga_r);
			str += fmt::format("omga_l: {}\n", omga_l);
			str += fmt::format("distance1: {}\n", distance1);
			str += fmt::format("distance2: {}\n", distance2);
			str += fmt::format("distance3: {}\n", distance3);
			str += fmt::format("distance4: {}\n", distance4);
			for (int k = 0; k < 9; ++k) str += fmt::format("imudata[{}]: {}\n", k, imudata[k]);
			str += fmt::format("timestamp: {}\n", timestamp);
			return str;
		}
	};
	CirArr<XQFrame> cirFrames;

private://1.Device
	string sname;
	asio::io_service io;
	asio::serial_port sport = asio::serial_port(io);

public://2.Config
	inline bool open(string spname)
	{
		sport.open(sname = spname);
		if (!sport.is_open()) return false;
		using namespace asio;
		sport.set_option(serial_port::baud_rate(115200));
		sport.set_option(serial_port::character_size(8));
		sport.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
		sport.set_option(serial_port::parity(serial_port::parity::none));
		sport.set_option(serial_port::flow_control(serial_port::flow_control::none));
		return true;
	}
	inline bool opened() { return sport.is_open(); }
	inline void close() { sport.close(); }
	inline string name() { return sname; }

public://3.Write
	const char heads[3] = { char(0xcd), char(0xeb), char(0xd7) };
	inline void setMode(char mode/*T/R/I*/)//T=debug   R=work   I=reset
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned: mode={}", mode); return; }
		char data[5] = { heads[0], heads[1], heads[2], char(0x01), mode };
		asio::write(sport, asio::buffer(data, 5));
	}
	inline void runCar(char action/*f/b/s/c/d*/, char velocity/*0~100*/)//f=move ahead   b=move back   s=brake   c=turn left   d=turn right
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned: action={}   velocity={}", action, int(velocity)); return; }
		char data[6] = { heads[0], heads[1], heads[2], char(0x02), action, velocity };
		asio::write(sport, asio::buffer(data, 6));
	}
	inline void runMotor(char action1/*F/B/S*/, char action2/*F/B/S*/, char velocity1/*0~100*/, char velocity2/*0~100*/)
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned: action1={}   action2={}   velocity1={}   velocity2={}", action1, action2, int(velocity1), int(velocity2)); return; }
		char data[13] = { heads[0], heads[1], heads[2], char(0x09), char(0x74), action1, action2, 0x53, 0x53, velocity1, velocity2, char(0x00), char(0x00) };
		asio::write(sport, asio::buffer(data, 13));
	}
	inline void runSensor(int action/*0=CalibIMU 1=EnableIMU 2=DisableIMU*/)
	{
		if (!sport.is_open()) { spdlog::critical("Serial port not openned: action={}", action == 0 ? "CalibIMU" : action == 1 ? "EnableIR" : "DisableIR"); return; }
		char data[6] = { heads[0], heads[1], heads[2], action == 0 ? char(0x01) : char(0x02), action == 0 ? char(0x43) : char(0x44), action == 2 ? char(0x00) : char(0x01) };
		asio::write(sport, asio::buffer(data, action == 0 ? 5 : 6));
	}

public://4.Read
};