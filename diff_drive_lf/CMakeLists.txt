cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_lf)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  joint_state_publisher_gui
  robot_state_publisher
  rviz
  xacro
  controller_manager
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  rqt_robot_steering
  diff_drive_controller
  roscpp
  rospy
  geometry_msgs
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)