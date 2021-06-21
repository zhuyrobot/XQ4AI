#include <cscv/base/acxx.h>
#include <ams_xq/msg/xq_frame.hpp>
#include <ams_xq/srv/set_string.hpp>
#include "XQ4IO.h"

class XQ4ROS2Server : public rclcpp::Node
{
public://XQ4Node
	XQ4IO ioXQ;
	XQ4ROS2Server(string port = "", string nn = "XQ4Server", string ns = "XQ") : Node(nn, ns) 
	{ 
		if (port.empty()) return;
		if (ioXQ.open(port)) spdlog::info("Open {} failed", port);
	}

public://ManuPort
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvClosePort = create_service<std_srvs::srv::Trigger>("ClosePort",
		[this](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)->void
		{
			res->message = "Close " + ioXQ.name() + " succeeded";
			if (ioXQ.opened()) ioXQ.close();
			else res->message = ioXQ.name() + " not opened";
		});

	//ros2 service call /XQ/OpenPort ams_xq/srv/SetString "{str: 'COM2'}"
	rclcpp::Service<ams_xq::srv::SetString>::SharedPtr srvManuPort = create_service<ams_xq::srv::SetString>("OpenPort",
		[this](const ams_xq::srv::SetString::Request::SharedPtr req, ams_xq::srv::SetString::Response::SharedPtr res)->void
		{
			if (ioXQ.opened()) res->msg = ioXQ.name() + " not closed";
			else res->msg = "Open " + ioXQ.name() + (ioXQ.open(req->str) ? " succeeded" : " failed and check whether to set right port name");
		});

public://ManuCar
	rclcpp::Service<ams_xq::srv::SetString>::SharedPtr srvSetMode = create_service<ams_xq::srv::SetString>("SetMode",
		[this](const ams_xq::srv::SetString::Request::SharedPtr req, ams_xq::srv::SetString::Response::SharedPtr res)->void
		{
			res->msg = "Done";
			ioXQ.setMode(req->str[0]);
		});

	//ros2 service call /XQ/OpenPort ams_xq/srv/SetString "{str: '1'}" //0/1/2
	rclcpp::Service<ams_xq::srv::SetString>::SharedPtr srvRunSensor = create_service<ams_xq::srv::SetString>("RunSensor",
		[this](const ams_xq::srv::SetString::Request::SharedPtr req, ams_xq::srv::SetString::Response::SharedPtr res)->void
		{
			res->msg = "Done";
			ioXQ.runSensor(int(req->str[0]) - 48);
		});

	//ros2 service call -r 1 /XQ/RunMotor ams_xq/srv/SetString "{str: 'FF22'}" //forward50 //SS22 for brake50
	rclcpp::Service<ams_xq::srv::SetString>::SharedPtr srvRunMotor = create_service<ams_xq::srv::SetString>("RunMotor",
		[this](const ams_xq::srv::SetString::Request::SharedPtr req, ams_xq::srv::SetString::Response::SharedPtr res)->void
		{
			res->msg = "Done";
			if (req->str.size() > 3) ioXQ.runMotor(req->str[0], req->str[1], req->str[2], req->str[3]);
			else res->msg = "Failed (too few params)";
		});

	//ros2 topic pub -r 1 /XQ/RunCar std_msgs/msg/String "{data: 'f2'}" //forward50 //s2 for brake50
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subRunCar = create_subscription<std_msgs::msg::String>("RunCar", 10,
		[&](const std_msgs::msg::String::SharedPtr val)->void 
		{ 
			if(val->data.size() > 1) ioXQ.runCar(val->data[0], val->data[1]); 
			else spdlog::error("Published data not enough");
		});

public://Update
	rclcpp::Publisher<ams_xq::msg::XQFrame>::SharedPtr pubXQFrame = create_publisher<ams_xq::msg::XQFrame>("XQFrame", 10);
    rclcpp::TimerBase::SharedPtr TimerXQFrame = create_wall_timer(50ms, [&]()->void
        {
			if(!ioXQ.opened()) { this_thread::sleep_for(2000ms); spdlog::error("No port opened"); return; }
			XQ4IO::XQFrame *frame = 0;
            ioXQ.getStatus(&frame);
			ams_xq::msg::XQFrame rosFrame;
			if(frame) memcpy(&rosFrame, frame, sizeof(rosFrame));
			else spdlog::warn("Invalid frame and it is possible to connect one wrong port if this happens continuously");
            pubXQFrame->publish(rosFrame);
        });

public:
	static void RunMe(int argc = 0, char** argv = 0) 
	{ 
		rclcpp::init(argc, argv);

		auto server = std::make_shared<XQ4ROS2Server>();
		vector<string> portnames;
		for (int k = 0; k < 100; ++k) portnames.push_back("/dev/ttyUSB" + std::to_string(k));
		for (int k = 0; k < 100; ++k) portnames.push_back("COM" + std::to_string(k));
		for (int k = 0; k < portnames.size(); ++k)
			if (server->ioXQ.open(portnames[k])) break;
		if(!server->ioXQ.opened()) { spdlog::error("Open port failed"); }
		else spdlog::info("{} has opened successfully", server->ioXQ.name());
		rclcpp::spin(server);
		
		rclcpp::shutdown();	
	}
};

int main(int argc, char** argv) { XQ4ROS2Server::RunMe(argc, argv); return 0; }

int main1(int argc, char** argv) { XQ4IO::SimXQ4("COM1"); return 0; }

int main2(int argc, char** argv) { XQ4IO::TestAsioTimerAndFunctionAndLambda(argc, argv); return 0; }