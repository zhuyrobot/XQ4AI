#include <cscv/base/acxx.h>
#include <ams_xq/msg/xq4_frame.hpp>
#include <ams_xq/srv/xq4_serve.hpp>
#include "XQ4IO.h"

class XQ4ROS : public rclcpp::Node
{
public://XQ4Node
	XQ4IO xq4io;
	XQ4ROS(string port = "", string nn = "XQ4Server", string ns = "XQ") : Node(nn, ns) 
	{ 
		if (port.empty()) stats.data = "Error: serial port cannot be empty and try again by service";
		else if (xq4io.open(port)) stats.data = fmt::format("Info: open {} done", port); 
		else stats.data = fmt::format("Error: open {} failed and try again by service", port);
		pubXQ4Status->publish(stats); spdlog::info(stats.data);
	}

public://PubStatus
	std_msgs::msg::String stats;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubXQ4Status = create_publisher<std_msgs::msg::String>("XQ4Status", 10);

public://ManuPort //ros2 service call /XQ/ClosePort std_srvs/srv/Trigger "{}"
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvClosePort = create_service<std_srvs::srv::Trigger>("ClosePort",
		[this](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)->void
		{		
			if (xq4io.opened()) { xq4io.close(); stats.data = fmt::format("Info: Close {} done", xq4io.name()); }
			else stats.data = fmt::format("Warn: {} not opened", xq4io.name());
			res->message = stats.data; pubXQ4Status->publish(stats); spdlog::info(stats.data);
		});

	//ros2 service call /XQ/OpenPort ams_xq/srv/XQ4Serve "{cmd: 'COM2'}"
	rclcpp::Service<ams_xq::srv::XQ4Serve>::SharedPtr srvManuPort = create_service<ams_xq::srv::XQ4Serve>("OpenPort",
		[this](const ams_xq::srv::XQ4Serve::Request::SharedPtr req, ams_xq::srv::XQ4Serve::Response::SharedPtr res)->void
		{
			if (xq4io.opened()) stats.data = fmt::format("Warn: {} not closed", xq4io.name());
			else if (xq4io.open(req->cmd)) stats.data = fmt::format("Info: open {} done", xq4io.name());
			else stats.data = fmt::format("Error: open {} failed", xq4io.name());
			res->msg = stats.data; pubXQ4Status->publish(stats); spdlog::info(stats.data);
		});

public://ManuCar //ros2 service call /XQ/SetMode ams_xq/srv/XQ4Serve "{cmd: 'T'}" //T/R/I
	rclcpp::Service<ams_xq::srv::XQ4Serve>::SharedPtr srvSetMode = create_service<ams_xq::srv::XQ4Serve>("SetMode",
		[this](const ams_xq::srv::XQ4Serve::Request::SharedPtr req, ams_xq::srv::XQ4Serve::Response::SharedPtr res)->void
		{
			if (req->cmd.size() != 1) stats.data = "Error: wrong format";
			else
			{
				xq4io.setMode(req->cmd[0]);
				stats.data = fmt::format("Info: SetMode({:x}) done", req->cmd[0]);
				res->msg = stats.data; pubXQ4Status->publish(stats); spdlog::info(stats.data);
			}
		});

	//ros2 service call /XQ/RunSensor ams_xq/srv/XQ4Serve "{cmd: '1'}" //0/1/2
	rclcpp::Service<ams_xq::srv::XQ4Serve>::SharedPtr srvRunSensor = create_service<ams_xq::srv::XQ4Serve>("RunSensor",
		[this](const ams_xq::srv::XQ4Serve::Request::SharedPtr req, ams_xq::srv::XQ4Serve::Response::SharedPtr res)->void
		{
			if (req->cmd.size() != 1) stats.data = "Error: wrong format";
			else
			{
				xq4io.runSensor(int(req->cmd[0]) - 48);
				stats.data = fmt::format("Info: RunSensor({:x}) done", req->cmd[0]);
				res->msg = stats.data; pubXQ4Status->publish(stats); spdlog::info(stats.data);
			}
		});

	//ros2 service call -r 1 /XQ/RunMotor ams_xq/srv/XQ4Serve "{cmd: 'FF22'}" //forward50 //SS22 for brake50
	rclcpp::Service<ams_xq::srv::XQ4Serve>::SharedPtr srvRunMotor = create_service<ams_xq::srv::XQ4Serve>("RunMotor",
		[this](const ams_xq::srv::XQ4Serve::Request::SharedPtr req, ams_xq::srv::XQ4Serve::Response::SharedPtr res)->void
		{
			if (req->cmd.size() != 3) stats.data = "Error: wrong format";
			else
			{
				xq4io.runMotor(req->cmd[0], req->cmd[1], req->cmd[2], req->cmd[3]);
				stats.data = fmt::format("Info: RunMotor({:x}-{:x}-{:x}) done", req->cmd[0], req->cmd[2], req->cmd[3]);
			}
			res->msg = stats.data; pubXQ4Status->publish(stats); spdlog::info(stats.data);
		});

	//ros2 topic pub -r 1 /XQ/RunCar std_msgs/msg/String "{data: 'f2'}" //forward50 //s2 for brake50
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subRunCar = create_subscription<std_msgs::msg::String>("RunCar", 10,
		[&](const std_msgs::msg::String::SharedPtr val)->void 
		{ 
			if (val->data.size() != 2) stats.data = "Error: wrong format";
			else xq4io.runCar(val->data[0], val->data[1]);
		});

public://Update
	rclcpp::Publisher<ams_xq::msg::XQ4Frame>::SharedPtr pubXQFrame = create_publisher<ams_xq::msg::XQ4Frame>("XQ4Frame", 10);
    rclcpp::TimerBase::SharedPtr TimerXQFrame = create_wall_timer(10ms, [&]()->void
        {
			if(!xq4io.opened()) 
			{ 
				this_thread::sleep_for(2000ms); 
				stats.data = "Error: no port opened. Please open it by service";
				pubXQ4Status->publish(stats); spdlog::info(stats.data);
				return;
			}
			XQ4IO::XQ4Frame *frame = 0;
            xq4io.getStatus(&frame);
			ams_xq::msg::XQ4Frame rosFrame;
			if(frame) memcpy(&rosFrame, frame, sizeof(rosFrame));
			else
			{
				stats.data = "Error: invalid frame \n\t(1)It is possible to connect one wrong port if this happens continuously";
				stats.data += fmt::format("\n\t(2) Close current {} and open the right one", xq4io.name());
				pubXQ4Status->publish(stats); spdlog::info(stats.data);
			}
            pubXQFrame->publish(rosFrame);
        });

public:
	static void RunMe(int argc = 0, char** argv = 0) 
	{ 
		rclcpp::init(argc, argv);

		auto server = std::make_shared<XQ4ROS>();
		vector<string> portnames;
		for (int k = 0; k < 100; ++k) portnames.push_back("/dev/ttyUSB" + std::to_string(k));
		for (int k = 0; k < 100; ++k) portnames.push_back("COM" + std::to_string(k));
		for (int k = 0; k < portnames.size(); ++k) if (server->xq4io.open(portnames[k])) break;
		rclcpp::spin(server);
		rclcpp::shutdown();	
	}
};

//int main(int argc, char** argv) { XQ4ROS::RunMe(argc, argv); return 0; }