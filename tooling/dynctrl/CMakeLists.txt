##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(DYNCTRL_TARGET dynctrl)
##########################################################################
add_executable(${DYNCTRL_TARGET}
  ../../src/dynamixel/DynamixelController.cpp
  ../../src/dynamixel/MX28Controller.cpp
  dynctrl.cpp
)
target_compile_options(${DYNCTRL_TARGET} PRIVATE -std=c++17)
target_link_libraries(${DYNCTRL_TARGET} dynamixel boost_program_options)
##########################################################################
