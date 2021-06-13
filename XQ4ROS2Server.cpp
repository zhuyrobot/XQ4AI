#include <cscv/base/atool.h>
#include <ams_xq/msg/xq_frame.hpp>
#include <ams_xq/srv/manu_port.hpp>
#include <ams_xq/srv/manu_car.hpp>
#include "XQ4IO.h"

class XQ4ROS2Server
{
private:
	XQ4IO ioXQ;

public://XQ4Node
	rclcpp::Node::SharedPtr nodXQ4Server = rclcpp::Node::make_shared("XQ4Server", "XQ");

public://ManuPort
	rclcpp::Service<ams_xq::srv::ManuPort>::SharedPtr srvManuPort = nodXQ4Server->create_service<ams_xq::srv::ManuPort>("ManuPort",
		[this](const ams_xq::srv::ManuPort::Request::SharedPtr req, ams_xq::srv::ManuPort::Response::SharedPtr res)->void
		{
			res->ret = true;
			if (req->action == false) ioXQ.close();
			else if (!ioXQ.opened()) res->ret = ioXQ.open(req->name);
			res->msg = (req->action ? "Open " : "Close ") + ioXQ.name() + (res->ret ? " succeeded" : " failed and check whether to set right port name");
		});

public://ManuCar
	rclcpp::Service<ams_xq::srv::ManuCar>::SharedPtr srvManuCar = nodXQ4Server->create_service<ams_xq::srv::ManuCar>("ManuCar",
		[this](const ams_xq::srv::ManuCar::Request::SharedPtr req, ams_xq::srv::ManuCar::Response::SharedPtr res)->void
		{
			res->ret = true;
			if (req->action == 0) ioXQ.setMode(req->data0);
			else if (req->action == 1) ioXQ.runCar(req->data0, req->data1);
			else if (req->action == 2) ioXQ.runMotor(req->data0, req->data1, req->data2, req->data3);
			else if (req->action == 3) ioXQ.runSensor(req->data0);
			else res->ret = false;
			res->msg = res->ret ? "Operation succeeded" : "Invalid command";
		});

public://Update
	rclcpp::Publisher<ams_xq::msg::XQFrame>::SharedPtr pubXQFrame = nodXQ4Server->create_publisher<ams_xq::msg::XQFrame>("XQFrame", 2);

    rclcpp::TimerBase::SharedPtr TimerXQFrame = nodXQ4Server->create_wall_timer(50ms, [&]()->void
        {
			XQ4IO::XQFrame *frame;
            ioXQ.getStatus(&frame);
			ams_xq::msg::XQFrame rosFrame;
			if(frame) memcpy(&rosFrame, frame, sizeof(rosFrame));
            pubXQFrame->publish(rosFrame);
        });

public:
	static void RunMe(int argc = 0, char** argv = 0) 
	{ 
		rclcpp::init(argc, argv);

		
		
		rclcpp::shutdown();	
	}
};

int main(int argc, char** argv) { XQ4ROS2Server::RunMe(argc, argv); return 0; }