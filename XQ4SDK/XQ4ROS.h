#include <cscv/base/acxx.h>
#include <ams_xq/msg/xq4_frame.hpp>
#include <ams_xq/srv/xq4_serve.hpp>
#include "XQ4IO.h"

class XQ4ROS : public rclcpp::Node
{
public://XQ4Node
	XQ4IO xq4io;
	XQ4ROS(string port = "", string nn = "XQ4Server", string ns = "XQ") : Node(nn, ns) {}

public://PubStatus
	std_msgs::msg::String stats;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubXQ4Status = create_publisher<std_msgs::msg::String>("XQ4Status", 10);

public://ManuPort //ros2 service call /XQ/ClosePort std_srvs/srv/Trigger "{}"
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvClosePort = create_service<std_srvs::srv::Trigger>("ClosePort",
		[this](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)->void
		{		
			if (xq4io.opened()) { xq4io.close(); stats.data = fmt::format("Close {} done", xq4io.name()); }
			else stats.data = fmt::format("{} not opened", xq4io.name());
			res->message = stats.data; pubXQ4Status->publish(stats); SPDLOG_INFO(stats.data);
		});

	//ros2 service call /XQ/OpenPort ams_xq/srv/XQ4Serve "{cmd: 'COM2'}"
	rclcpp::Service<ams_xq::srv::XQ4Serve>::SharedPtr srvManuPort = create_service<ams_xq::srv::XQ4Serve>("OpenPort",
		[this](const ams_xq::srv::XQ4Serve::Request::SharedPtr req, ams_xq::srv::XQ4Serve::Response::SharedPtr res)->void
		{
			if (xq4io.opened()) stats.data = fmt::format("{} not closed", xq4io.name());
			else if (xq4io.open(req->cmd)) stats.data = fmt::format("Open {} done", xq4io.name());
			else stats.data = fmt::format("Open {} failed", xq4io.name());
			res->msg = stats.data; pubXQ4Status->publish(stats); SPDLOG_INFO(stats.data);
		});

public://ManuCar //ros2 service call /XQ/SetMode ams_xq/srv/XQ4Serve "{cmd: 'T'}" //T/R/I
	rclcpp::Service<ams_xq::srv::XQ4Serve>::SharedPtr srvSetMode = create_service<ams_xq::srv::XQ4Serve>("SetMode",
		[this](const ams_xq::srv::XQ4Serve::Request::SharedPtr req, ams_xq::srv::XQ4Serve::Response::SharedPtr res)->void
		{
			if (req->cmd.size() != 1) stats.data = "Wrong format";
			else
			{
				xq4io.setMode(req->cmd[0]);
				stats.data = fmt::format("SetMode({:x}) done", req->cmd[0]);
				res->msg = stats.data; pubXQ4Status->publish(stats); SPDLOG_INFO(stats.data);
			}
		});

	//ros2 service call /XQ/RunSensor ams_xq/srv/XQ4Serve "{cmd: '1'}" //0/1/2
	rclcpp::Service<ams_xq::srv::XQ4Serve>::SharedPtr srvRunSensor = create_service<ams_xq::srv::XQ4Serve>("RunSensor",
		[this](const ams_xq::srv::XQ4Serve::Request::SharedPtr req, ams_xq::srv::XQ4Serve::Response::SharedPtr res)->void
		{
			if (req->cmd.size() != 1) stats.data = "Wrong format";
			else
			{
				xq4io.runSensor(int(req->cmd[0]) - 48);
				stats.data = fmt::format("RunSensor({:x}) done", req->cmd[0]);
				res->msg = stats.data; pubXQ4Status->publish(stats); SPDLOG_INFO(stats.data);
			}
		});

	//ros2 service call -r 1 /XQ/RunMotor ams_xq/srv/XQ4Serve "{cmd: 'FF22'}" //forward50 //SS22 for brake50
	rclcpp::Service<ams_xq::srv::XQ4Serve>::SharedPtr srvRunMotor = create_service<ams_xq::srv::XQ4Serve>("RunMotor",
		[this](const ams_xq::srv::XQ4Serve::Request::SharedPtr req, ams_xq::srv::XQ4Serve::Response::SharedPtr res)->void
		{
			if (req->cmd.size() != 3) stats.data = "Wrong format";
			else
			{
				xq4io.runMotor(req->cmd[0], req->cmd[1], req->cmd[2], req->cmd[3]);
				stats.data = fmt::format("RunMotor({:x}-{:x}-{:x}) done", req->cmd[0], req->cmd[2], req->cmd[3]);
			}
			res->msg = stats.data; pubXQ4Status->publish(stats); SPDLOG_INFO(stats.data);
		});

	//ros2 topic pub -r 1 /XQ/RunCar std_msgs/msg/String "{data: 'f2'}" //forward50 //s2 for brake50
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subRunCar = create_subscription<std_msgs::msg::String>("RunCar", 10,
		[&](const std_msgs::msg::String::SharedPtr val)->void 
		{ 
			if (val->data.size() != 2) stats.data = "Wrong format";
			else xq4io.runCar(val->data[0], val->data[1]);
		});

public://Update
	rclcpp::Publisher<ams_xq::msg::XQ4Frame>::SharedPtr pubXQFrame = create_publisher<ams_xq::msg::XQ4Frame>("XQ4Frame", 10);
    rclcpp::TimerBase::SharedPtr TimerXQFrame = create_wall_timer(10ms, [&]()->void
        {
			//1.CheckPortOpen
			if(!xq4io.opened()) 
			{ 
				this_thread::sleep_for(1000ms); 
				stats.data = "No port opened.XQ4ROS will try open one every 2000ms.You can open it by service if knowing port name";
				pubXQ4Status->publish(stats); SPDLOG_ERROR(stats.data);
				CloseCurrentOpenAnotherSPort(xq4io);
				return;
			}

			//2.CheckCanRead
			static int64_t nFailedFrame = 0;
			XQ4IO::XQ4Frame *frame = 0;
            xq4io.getStatus(&frame);
			ams_xq::msg::XQ4Frame rosFrame;
			if (frame == 0)
			{
				++nFailedFrame;
				stats.data = fmt::format("Failed to get XQ4Status from {}", xq4io.name());
				if (nFailedFrame > 3)
				{
					stats.data += ".It is possible to connect one wrong port.XQ4ROS will close current one and try open another one.";
					CloseCurrentOpenAnotherSPort(xq4io);
					nFailedFrame = 0;
				}
				pubXQ4Status->publish(stats); SPDLOG_ERROR(stats.data);
				return;
			}
			else nFailedFrame = 0;

			//3.CheckBeXQ4Port
			static int64_t nInvalidFrame = 0;
			memcpy(&rosFrame, frame, sizeof(rosFrame));
			if (rosFrame.status < 0 || rosFrame.status > 1 || rosFrame.power < 8.f || rosFrame.power > 13.f)
			{
				++nInvalidFrame;
				stats.data = fmt::format("Invalid frame from {}", xq4io.name());
				if (nFailedFrame > 10)
				{
					stats.data += ".It is possible to connect one wrong port.XQ4ROS will close current one and try open another one.";
					CloseCurrentOpenAnotherSPort(xq4io);
					nInvalidFrame = 0;
				}
				pubXQ4Status->publish(stats); SPDLOG_ERROR(stats.data);
			}
			else nInvalidFrame = 0;

			//4.PublishFrame
            pubXQFrame->publish(rosFrame);
        });

public:
	static void CloseCurrentOpenAnotherSPort(XQ4IO& xq4io, string priorSPort = "")
	{
		static int nport = 0;
		if (xq4io.opened()) xq4io.close();
		if (!priorSPort.empty()) if (xq4io.open(priorSPort)) return;
		for (int k = 0; k < 100; ++k, ++nport)
		{
			if (xq4io.open(fmt::format("/dev/ttyUSB{}", nport))) break;
			if (xq4io.open(fmt::format("COM{}", nport))) break;
			if (nport > 99) nport = 0;
		}
	}

	static void RunMe(int argc = 0, char** argv = 0)
	{
		rclcpp::init(argc, argv);
		auto server = std::make_shared<XQ4ROS>();
		CloseCurrentOpenAnotherSPort(server->xq4io, argc > 1 ? argv[1] : "");
		rclcpp::spin(server);
		rclcpp::shutdown();
	}
};

//int main(int argc, char** argv) { XQ4ROS::RunMe(argc, argv); return 0; }