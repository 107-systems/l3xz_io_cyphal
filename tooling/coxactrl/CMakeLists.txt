##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(COXACTRL_TARGET coxactrl)
##########################################################################
add_executable(${COXACTRL_TARGET}
  ../../src/driver/dynamixel/Dynamixel.cpp
  ../../src/driver/dynamixel/MX28.cpp
  ../../src/Coxa.cpp
  coxactrl.cpp
)
target_compile_options(${COXACTRL_TARGET} PRIVATE -std=c++17)
target_link_libraries(${COXACTRL_TARGET} dynamixel boost_program_options)
##########################################################################
