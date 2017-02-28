cmake_minimum_required(VERSION 2.8.3)
project(iai_motion_projection)

find_package(catkin REQUIRED COMPONENTS
  iai_naive_kinematics_sim
  rospy
  std_msgs
  message_generation
  message_runtime
)

add_service_files(DIRECTORY srv
  FILES
  TriggerProjection.srv)

generate_messages()

catkin_package(
 CATKIN_DEPENDS iai_naive_kinematics_sim rospy std_msgs message_runtime)
