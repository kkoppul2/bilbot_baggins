cmake_minimum_required(VERSION 2.8.3)
project(bilbot_drive)

find_package(catkin REQUIRED COMPONENTS
  controller_manager diagnostic_updater geometry_msgs hardware_interface bilbot_msgs 
  realtime_tools roscpp roslaunch roslint sensor_msgs std_msgs 
  topic_tools)
find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(bilbot_drive_node
  src/bilbot_base.cpp
  src/bilbot_hardware.cpp)
target_link_libraries(bilbot_drive_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(bilbot_drive_node bilbot_msgs_gencpp)

install(TARGETS bilbot_drive_node 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

roslaunch_add_file_check(launch/drive.launch)
roslint_cpp()
roslint_add_test()