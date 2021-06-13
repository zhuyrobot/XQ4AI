#include <cscv/base/atool.h>
#include <ams_xq/msg/xq_frame.hpp>
#include "XQ4IO.h"

class XQ4ROS2Server
{
private:
	XQ4IO ioXQ;

public:
	rclcpp::Node::SharedPtr nodControl = rclcpp::Node::make_shared("nodControl", "XQ");

	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subWorkMode = 
		nodControl->create_subscription<std_msgs::msg::Int8>("setMode", 2, [this](std_msgs::msg::Int8::SharedPtr val)->void { ioXQ.setMode(char(val->data)); });

	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subRunCar =
		nodControl->create_subscription<std_msgs::msg::Int16>("runCar", 2, [this](std_msgs::msg::Int16::SharedPtr val)->void { ioXQ.runCar(char(val->data & 0xFF00), char(val->data & 0xFF)); });

	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subRunMotor =
		nodControl->create_subscription<std_msgs::msg::Int32>("runMotor", 2, [this](std_msgs::msg::Int32::SharedPtr val)->void { ioXQ.runMotor(char(val->data & 0xFF000000), char(val->data & 0xFF0000), char(val->data & 0xFF00), char(val->data & 0xFF)); });

	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subRunSensor =
		nodControl->create_subscription<std_msgs::msg::Int8>("runSensor", 2, [this](std_msgs::msg::Int8::SharedPtr val)->void { ioXQ.runSensor(char(val->data)); });

public:
	rclcpp::Node::SharedPtr nodUpdate = rclcpp::Node::make_shared("nodUpdate", "XQ");

	rclcpp::Publisher<ams_xq::msg::XQFrame>::SharedPtr pubXQFrame = nodUpdate->create_publisher<ams_xq::msg::XQFrame>("rawFrame", 2);

    rclcpp::TimerBase::SharedPtr TimerXQFrame = nodUpdate->create_wall_timer(50ms, [&]()->void
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