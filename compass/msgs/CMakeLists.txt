cmake_minimum_required(VERSION 2.8.3)
project(dfcompass_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  status.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package()
