#include <cscv/base/acxx.h>
#include <turtlesim/srv/Kill.hpp>
#include <turtlesim/srv/Spawn.hpp>
#include "XQ4IO.h"

class XQ4ROS2Server
{
public://XQ4Node
	XQ4IO ioXQ;
	rclcpp::Node::SharedPtr nodXQ4Server = rclcpp::Node::make_shared("XQ4Server", "XQ");

public://ManuPort
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvClosePort = nodXQ4Server->create_service<std_srvs::srv::Trigger>("ClosePort",
		[this](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)->void
		{
			res->message = "Close " + ioXQ.name() + " succeeded";
			if (ioXQ.opened()) ioXQ.close();
			else res->message = ioXQ.name() + " not opened";
		});

	rclcpp::Service<turtlesim::srv::Spawn>::SharedPtr srvManuPort = nodXQ4Server->create_service<turtlesim::srv::Spawn>("OpenPort",
		[this](const turtlesim::srv::Spawn::Request::SharedPtr req, turtlesim::srv::Spawn::Response::SharedPtr res)->void
		{
			if (ioXQ.opened()) res->name = ioXQ.name() + " not closed";
			else 
			{
				bool sc = ioXQ.open(req->name);
				res->name = "Open " + ioXQ.name() + (sc ? " succeeded" : " failed and check whether to set right port name");
			}
		});

public://ManuCar
	rclcpp::Service<turtlesim::srv::Spawn>::SharedPtr srvSetModeOrSensor = nodXQ4Server->create_service<turtlesim::srv::Spawn>("SetModeOrSensor",
		[this](const turtlesim::srv::Spawn::Request::SharedPtr req, turtlesim::srv::Spawn::Response::SharedPtr res)->void
		{
			res->name = "Done";
			if (req->name[0] == 0 || req->name[0] == 1 || req->name[0] == 2) ioXQ.setMode(req->name[0]);
			else ioXQ.runSensor(req->name[0]);
		});

	rclcpp::Service<turtlesim::srv::Spawn>::SharedPtr srvRunMotor = nodXQ4Server->create_service<turtlesim::srv::Spawn>("RunMotor",
		[this](const turtlesim::srv::Spawn::Request::SharedPtr req, turtlesim::srv::Spawn::Response::SharedPtr res)->void
		{
			res->name = "Done";
			if (req->name.size() < 3) ioXQ.runMotor(req->name[0], req->name[1], req->name[2], req->name[3]);
			else res->name = "Failed (too few params)";
		});

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subMotion = nodXQ4Server->create_subscription<std_msgs::msg::String>("RunCar", 2,
		[&](const std_msgs::msg::String::SharedPtr val)->void { ioXQ.runCar(val->data[0], val->data[1]); });

public://Update
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubXQFrame = nodXQ4Server->create_publisher<std_msgs::msg::String>("XQFrame", 2);
    rclcpp::TimerBase::SharedPtr TimerXQFrame = nodXQ4Server->create_wall_timer(50ms, [&]()->void
        {
			if(!ioXQ.opened()) { this_thread::sleep_for(2000ms); return; }
			XQ4IO::XQFrame *frame = 0;
            ioXQ.getStatus(&frame);
			std_msgs::msg::String rosFrame; 
			rosFrame.data.assign(sizeof(XQ4IO::XQFrame), 0);
			if(frame) memcpy(rosFrame.data.data(), frame, sizeof(XQ4IO::XQFrame));
			else spdlog::warn("Invalid frame and if this happens often:\n\t(1)not open car board\n\t(2)connect wrong port\n\t(3)");
            pubXQFrame->publish(rosFrame);
        });

public:
	static void RunMe(int argc = 0, char** argv = 0) 
	{ 
		rclcpp::init(argc, argv);

		XQ4ROS2Server server;
		vector<string> portnames;
		for (int k = 0; k < 100; ++k) portnames.push_back("/dev/ttyUSB" + std::to_string(k));
		for (int k = 0; k < 100; ++k) portnames.push_back("COM" + std::to_string(k));
		for (int k = 0; k < portnames.size(); ++k)
			if (server.ioXQ.open(portnames[k])) break;
		if(!server.ioXQ.opened()) { spdlog::error("Open port failed"); return; }
		else spdlog::info("{} has opened successfully", server.ioXQ.name());
		rclcpp::spin(server.nodXQ4Server);
		
		rclcpp::shutdown();	
	}
};

int main(int argc, char** argv) { XQ4ROS2Server::RunMe(argc, argv); return 0; }

int main1(int argc, char** argv) { XQ4IO::SimXQ4("COM1"); return 0; }

int main2(int argc, char** argv) { XQ4IO::TestAsioTimerAndFunctionAndLambda(argc, argv); return 0; }