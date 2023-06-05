cmake_minimum_required(VERSION 3.0.2)
project(lidar_processor)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_srvs
)

catkin_package(
	CATKIN_DEPENDS rospy std_srvs
)

catkin_install_python(PROGRAMS
	src/laser_scan_processor.py
	src/service_caller.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

