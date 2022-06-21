#######################################################################################
cmake_minimum_required(VERSION 3.3.0)
#######################################################################################
project(l3xz)
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
##########################################################################
find_package(orocos_kdl REQUIRED)
##########################################################################
add_subdirectory(external/libcanard/current)
add_subdirectory(external/libdynamixel/current)
add_subdirectory(external/libo1heap/current)
add_subdirectory(external/libsocketcan)
add_subdirectory(external/libuavcan
                 ${CMAKE_BINARY_DIR}/l3xz/external/libuavcan
)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  external/libuavcan/libuavcan/include
  external/libuavcan/libuavcan/include/dsdlc_generated
  external/libuavcan_linux/include
)

# libuavcan (libdronecan) build configuration
add_compile_definitions(
  UAVCAN_CPP_VERSION=UAVCAN_CPP11
)

add_executable(${PROJECT_NAME}_node
  src/common/actuator/interface/AnglePositionActuator.cpp
  src/common/kinematic/Engine.cpp
  src/common/kinematic/FK_Output.cpp
  src/common/kinematic/IK_Output.cpp
  src/common/sensor/interface/AnglePositionSensor.cpp
  src/common/sensor/interface/BumperSensor.cpp
  src/common/threading/ThreadBase.cpp
  src/common/threading/ThreadStats.cpp
  src/driver/dynamixel/Dynamixel.cpp
  src/driver/dynamixel/MX28.cpp
  src/driver/orel20/Orel20.cpp
  src/driver/ssc32/SSC32.cpp
  src/phy/dronecan/platform_linux.cpp
  src/phy/opencyphal/Node.cpp
  src/phy/opencyphal/SocketCAN.cpp
  src/phy/serial/AsyncSerial.cpp
  src/ros/RosBrigdeNode.cpp
  src/gait/state/Walking.cpp
  src/gait/state/Calibrate.cpp
  src/gait/state/Standing.cpp
  src/gait/state/StandUp.cpp
  src/gait/state/Turning.cpp
  src/gait/GaitController.cpp
  src/gait/GaitControllerInput.cpp
  src/gait/GaitControllerOutput.cpp
  src/head/state/Init.cpp
  src/head/state/Teleop.cpp
  src/head/HeadController.cpp
  src/head/HeadControllerOutput.cpp
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
  uavcan
)
#######################################################################################
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Werror -pedantic)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs)
#######################################################################################
add_subdirectory(tooling/fwlegctrl)
add_subdirectory(tooling/orocos)
add_subdirectory(tooling/ssc32ctrl)
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
