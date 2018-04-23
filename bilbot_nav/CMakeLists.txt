cmake_minimum_required(VERSION 2.8.3)
project(bilbot_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  roscpp
  sensor_msgs
  tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(action_pub_node
  src/nav_goals.cpp
)
target_link_libraries(action_pub_node
  ${catkin_LIBRARIES}
)

add_dependencies(action_pub_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

install(TARGETS action_pub_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch/nav.launch)
