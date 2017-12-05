cmake_minimum_required(VERSION 2.8.3)
project(tigrillo_io)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  rospy
  std_msgs
)

add_service_files(DIRECTORY srv
                  FILES
                  Frequency.srv)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS bin/uartd bin/i2cd
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



