cmake_minimum_required(VERSION 2.8.3)
project(yumi_description)

find_package(catkin REQUIRED)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	rospy
	controller_manager
	joint_state_controller
	position_controllers
	)

install(
	DIRECTORY config launch meshes urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
