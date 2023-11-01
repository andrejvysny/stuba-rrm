cmake_minimum_required(VERSION 3.0.2)
project(rrm_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  genmsg
)

add_service_files(FILES Move.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()
