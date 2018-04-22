cmake_minimum_required(VERSION 2.8.3)
project(bilbot_hardware)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# add_compile_options(-fno-sanitize-recover=all


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  bilbot_msgs
  std_msgs
  realtime_tools
  dynamic_reconfigure
  tf
)

find_package(Boost REQUIRED COMPONENTS thread chrono)

find_library(PIGPIO_LIB pigpiod_if2 /usr/lib/)
find_library(REALTIME_LIB rt /usr/lib/arm-linux-gnueabihf/)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/controller.cfg
)

catkin_package()


add_executable(motor_controller_node
  src/motor_output.cpp
  src/motor_controller.cpp)
target_link_libraries(motor_controller_node ${REALTIME_LIB} ${PIGPIO_LIB} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(motor_controller_node 
  ${catkin_EXPORTED_TARGETS} 
  bilbot_msgs_gencpp 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_executable(rotary_encoder_node
 src/wheel_state.cpp
 src/rotary_encoder.cpp
)
target_link_libraries(rotary_encoder_node ${REALTIME_LIB} ${PIGPIO_LIB} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(rotary_encoder_node 
  ${catkin_EXPORTED_TARGETS} 
  bilbot_msgs_gencpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(ultrasonic_node
 src/ultrasonic_sensor.cpp
 src/ultrasonic.cpp
)
target_link_libraries(ultrasonic_node ${REALTIME_LIB} ${PIGPIO_LIB} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ultrasonic_node 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(message_merge_node
  src/merge.cpp
)
target_link_libraries(message_merge_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(message_merge_node 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(laser_pub_node
  src/laser.cpp
)
target_link_libraries(laser_pub_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(laser_pub_node 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

install(TARGETS laser_pub_node message_merge_node rotary_encoder_node motor_controller_node ultrasonic_node 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch/hardware.launch)
roslint_cpp()
roslint_add_test()

