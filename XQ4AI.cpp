#include "XQ4Studio.h"

int main(int argc, char** argv)
{
	std::map<string, function<void(int, char**)>> funcs;
	funcs["XQ4ROS"] = XQ4ROS::RunMe;
	funcs["XQ4Studio"] = XQ4Studio::RunMe;
	funcs["XQ4Simulator"] = [](int argc, char** argv)->void
	{
		if (argc < 2) spdlog::error("Format: appname sport_name");
		else XQ4IO::XQ4Sim(argv[1]);
	};
	if (argc < 2)
	{
		string str("Enter one of following commands in the terminal with ros2 setup.bash/bat configuration:");
		for (std::map<string, function<void(int, char**)>>::iterator it = funcs.begin(); it != funcs.end(); ++it)
			str += fmt::format("\n\tXQ4AI {}", it->first);
		cout << str << "\n\nNow press any key to start XQ4Studio"; getchar();
		XQ4Studio::RunMe(argc, argv);
	}
	else
	{
		int argn = argc - 1;
		vector<char*> argp; for (int k = 1; k < argc; ++k) argp.push_back(argv[k]);
		funcs[argv[1]](argn, argp.data());
	}
	return 0;
}