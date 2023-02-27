#######################################################################################
cmake_minimum_required(VERSION 3.3.0)
#######################################################################################
project(ros2_cyphal_bridge)
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(common_interfaces REQUIRED)
##########################################################################
add_subdirectory(external/107-Arduino-Cyphal)
add_subdirectory(external/libsocketcan)
##########################################################################
include_directories(
  include
)
##########################################################################
add_executable(${PROJECT_NAME}_node
  src/CanManager.cpp
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_link_libraries(${PROJECT_NAME}_node
  cyphal++ socketcan
)
#######################################################################################
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Werror -pedantic)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)
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
