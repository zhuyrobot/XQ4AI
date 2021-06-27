1.编译XQ4ROS2组件
	(1)复制ams_xq到/root/app/ros2ex
	(2)新建/root/app/ros2ex/out(若已存在则将清空目录内的所有内容)
	(3)打开终端并进入/root/app/ros2ex/out执行：source /opt/ros/foxy/setup.bash && colcon build --merge-install --base-paths ..

2.编译XQ4服务及相关测试用例
	(1)复制仅头文件的依赖(如CLI/Sophus/CSCV/Cereal等)到/root/app/include
	(2)复制此工程到/root/app/XQ4RC(此目录内应包括CMakeList.txt和README.txt)
	(3)新建/root/app/XQ4RC/out(若已存在则将清空目录内的所有内容)
	(4)打开终端执行：source /root/app/ros2ex/out/install/setup.bash	
	(5)进入/root/app/XQ4RC/out执行：cmake .. -DCOMMON_CMAKE=xxx && make
	