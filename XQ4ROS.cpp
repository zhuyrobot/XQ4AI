#include "XQ4IO.h"

class XQ4RC : public QWidget
{
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

private:
	asio::io_service io;
	asio::serial_port sp = asio::serial_port(io);
	const char heads[3] = { char(0xcd), char(0xeb), char(0xd7) };
	void setMode(char mode/*T/R/I*/)//T=debug   R=work   I=reset
	{
		if(!sp.is_open()) { spdlog::critical("Serial port not openned: mode={}", mode); return; }
		char data[5] = { heads[0], heads[1], heads[2], char(0x01), mode };
		asio::write(sp, asio::buffer(data, 5));
	}
	void runCar(char action/*f/b/s/c/d*/, char velocity/*0~100*/)//f=move ahead   b=move back   s=brake   c=turn left   d=turn right
	{
		if (!sp.is_open()) { spdlog::critical("Serial port not openned: action={}   velocity={}", action, int(velocity)); return; }
		char data[6] = { heads[0], heads[1], heads[2], char(0x02), action, velocity };
		asio::write(sp, asio::buffer(data, 6));
	}
	void runMotor(char action1/*F/B/S*/, char action2/*F/B/S*/, char velocity1/*0~100*/, char velocity2/*0~100*/)
	{
		if (!sp.is_open()) { spdlog::critical("Serial port not openned: action1={}   action2={}   velocity1={}   velocity2={}", action1, action2, int(velocity1), int(velocity2)); return; }
		char data[13] = { heads[0], heads[1], heads[2], char(0x09), char(0x74), action1, action2, 0x53, 0x53, velocity1, velocity2, char(0x00), char(0x00) };
		asio::write(sp, asio::buffer(data, 13));
	}
	void runSensor(int action/*0=CalibIMU 1=EnableIMU 2=DisableIMU*/)
	{
		if (!sp.is_open()) { spdlog::critical("Serial port not openned: action={}", action == 0 ? "CalibIMU" : action == 1 ? "EnableIR" : "DisableIR"); return; }
		char data[6] = { heads[0], heads[1], heads[2], action == 0 ? char(0x01) : char(0x02), action == 0 ? char(0x43) : char(0x44), action == 2 ? char(0x00) : char(0x01) };
		asio::write(sp, asio::buffer(data, action == 0 ? 5 : 6));
	}

private:
	rclcpp::Node::SharedPtr nodCtrl = rclcpp::Node::make_shared("nodeWrite", "XQ");

	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subWorkMode = 
		nodCtrl->create_subscription<std_msgs::msg::Int8>("setMode", 2, [this](std_msgs::msg::Int8::SharedPtr val)->void { setMode(char(val->data)); });

	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subRunCar =
		nodCtrl->create_subscription<std_msgs::msg::Int16>("runCar", 2, [this](std_msgs::msg::Int16::SharedPtr val)->void { runCar(char(val->data & 0xFF00), char(val->data & 0xFF)); });

	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subRunMotor =
		nodCtrl->create_subscription<std_msgs::msg::Int32>("runMotor", 2, [this](std_msgs::msg::Int32::SharedPtr val)->void { runMotor(char(val->data & 0xFF000000), char(val->data & 0xFF0000), char(val->data & 0xFF00), char(val->data & 0xFF)); });

	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subRunSensor =
		nodCtrl->create_subscription<std_msgs::msg::Int8>("runSensor", 2, [this](std_msgs::msg::Int8::SharedPtr val)->void { runSensor(char(val->data)); });

public:
	static void RunMe(int argc = 0, char** argv = 0) {  }

};

int main(int argc, char** argv) { XQ4RC::RunMe(argc, argv); return 0; }