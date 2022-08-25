#######################################################################################
cmake_minimum_required(VERSION 3.3.0)
#######################################################################################
project(l3xz_io)
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(l3xz_gait_ctrl REQUIRED)
find_package(l3xz_head_ctrl REQUIRED)
##########################################################################
find_package(orocos_kdl REQUIRED)
##########################################################################
add_subdirectory(external/libcanard/current)
add_subdirectory(external/libdynamixel/current)
add_subdirectory(external/libo1heap/current)
add_subdirectory(external/libsocketcan)
##########################################################################
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
)
##########################################################################
add_executable(${PROJECT_NAME}_node
  src/driver/dynamixel/Dynamixel.cpp
  src/driver/dynamixel/MX28.cpp
  src/driver/ssc32/SSC32.cpp
  src/glue/DynamixelAnglePositionWriter.cpp
  src/glue/DynamixelAnglePositionReader.cpp
  src/glue/DynamixelServoName.cpp
  src/glue/LegController.cpp
  src/glue/PumpController.cpp
  src/glue/ValveController.cpp
  src/phy/opencyphal/Node.cpp
  src/phy/opencyphal/SocketCAN.cpp
  src/phy/serial/AsyncSerial.cpp
  src/IoNode.cpp
  src/main.cpp
)
#######################################################################################
target_link_libraries(${PROJECT_NAME}_node
  canard
  dynamixel
  boost_system
  boost_thread
  o1heap
  orocos-kdl
  pthread
  socketcan
)
#######################################################################################
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Werror -pedantic)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp l3xz_gait_ctrl l3xz_head_ctrl)
#######################################################################################
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
