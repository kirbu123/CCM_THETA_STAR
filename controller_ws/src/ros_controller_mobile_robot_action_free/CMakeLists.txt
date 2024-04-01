cmake_minimum_required(VERSION 3.0.2)
project(control_mobile_robot_oop)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  image_transport
  genmsg
  actionlib_msgs
  actionlib
  tf
  nav_msgs
  visualization_msgs
)
# Generate actions in the 'action' folder
add_action_files(
  FILES
  Control.action
  Simulatecollision.action
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  actionlib_msgs
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
)

## Declare a C++ executable
add_executable(control_mobile_robot_oop src/main_action.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(control_mobile_robot_oop  ${catkin_LIBRARIES})
add_dependencies(control_mobile_robot_oop ${PROJECT_NAME}_generate_messages_cpp)

