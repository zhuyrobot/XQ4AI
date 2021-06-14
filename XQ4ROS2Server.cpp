#include <cscv/base/acxx.h>
#include <ams_xq/msg/xq_frame.hpp>
#include <ams_xq/srv/open_port.hpp>
#include <ams_xq/srv/set_char.hpp>
#include <ams_xq/srv/set_chars.hpp>
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
			res->success = true;
			res->message = "Close " + ioXQ.name() + " succeeded";
			if (ioXQ.opened()) ioXQ.close();
			else res->message = ioXQ.name() + " not opened";
		});

	rclcpp::Service<ams_xq::srv::OpenPort>::SharedPtr srvManuPort = nodXQ4Server->create_service<ams_xq::srv::OpenPort>("OpenPort",
		[this](const ams_xq::srv::OpenPort::Request::SharedPtr req, ams_xq::srv::OpenPort::Response::SharedPtr res)->void
		{
			res->success = true;
			if (ioXQ.opened()) res->message = ioXQ.name() + " not closed";
			else 
			{
				res->success = ioXQ.open(req->name);
				res->message = "Open " + ioXQ.name() + (res->success ? " succeeded" : " failed and check whether to set right port name");
			}
		});

public://ManuCar
	rclcpp::Service<ams_xq::srv::SetChar>::SharedPtr srvSetModeOrSensor = nodXQ4Server->create_service<ams_xq::srv::SetChar>("SetModeOrSensor",
		[this](const ams_xq::srv::SetChar::Request::SharedPtr req, ams_xq::srv::SetChar::Response::SharedPtr res)->void
		{
			res->success = true;
			res->message = "Done";
			if (req->data == 0 || req->data == 1 || req->data == 2) ioXQ.setMode(req->data);
			else ioXQ.runSensor(req->data);
		});

	rclcpp::Service<ams_xq::srv::SetChars>::SharedPtr srvRunMotor = nodXQ4Server->create_service<ams_xq::srv::SetChars>("RunMotor",
		[this](const ams_xq::srv::SetChars::Request::SharedPtr req, ams_xq::srv::SetChars::Response::SharedPtr res)->void
		{
			res->success = true;
			res->message = "Done";
			if (req->data.size() < 3) ioXQ.runMotor(req->data[0], req->data[1], req->data[2], req->data[3]);
			else { res->success = false; res->message = "Failed (too few params)"; }
		});

	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subMotion = nodXQ4Server->create_subscription<std_msgs::msg::Int16>("RunCar", 2,
		[&](const std_msgs::msg::Int16::SharedPtr val)->void { ioXQ.runCar(char(val && 0xFF00), char(val && 0xFF)); });

public://Update
	rclcpp::Publisher<ams_xq::msg::XQFrame>::SharedPtr pubXQFrame = nodXQ4Server->create_publisher<ams_xq::msg::XQFrame>("XQFrame", 2);
    rclcpp::TimerBase::SharedPtr TimerXQFrame = nodXQ4Server->create_wall_timer(50ms, [&]()->void
        {
			if(!ioXQ.opened()) { this_thread::sleep_for(2000ms); return; }
			XQ4IO::XQFrame *frame = 0;
            ioXQ.getStatus(&frame);
			ams_xq::msg::XQFrame rosFrame;
			if(frame) memcpy(&rosFrame, frame, sizeof(rosFrame));
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