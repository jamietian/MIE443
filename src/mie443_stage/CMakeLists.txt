cmake_minimum_required(VERSION 2.8.3)
project(mie443_stage)

find_package(OpenCV 3.3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  	actionlib
  	move_base_msgs
  	roscpp
		cv_bridge
		image_transport
		gazebo_ros
)

find_package(orocos_kdl REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
