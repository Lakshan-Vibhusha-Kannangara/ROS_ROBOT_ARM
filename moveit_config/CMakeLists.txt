cmake_minimum_required(VERSION 3.0.2)
project(moveit_config)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  geometry_msgs
  sensor_msgs
  eigen_conversions
  tf2
  tf2_eigen
  moveit_ros_planning_interface  
  moveit_ros_move_group
	moveit_visual_tools
)
find_package(Eigen3 REQUIRED)
catkin_package(
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
add_executable(move_node src/move_group.cpp)
target_link_libraries(move _node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
