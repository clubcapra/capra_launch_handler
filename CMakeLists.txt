cmake_minimum_required(VERSION 3.0.2)
project(capra_launch_handler)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
    FILES
    LaunchMsg.msg
)
 add_service_files(
   FILES
   LaunchRequest.srv
   LaunchListRequest.srv
  )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

