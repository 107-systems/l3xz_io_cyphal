##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(SWEEPCTRL_TARGET sweepctrl)
##########################################################################
add_executable(${SWEEPCTRL_TARGET}
  ../../src/common/threading/ThreadBase.cpp
  ../../src/common/threading/ThreadStats.cpp
  ../../src/driver/sweep/SweepThread.cpp
  sweepctrl.cpp
)
target_compile_options(${SWEEPCTRL_TARGET} PRIVATE -std=c++17)
target_link_libraries(${SWEEPCTRL_TARGET} sweep boost_program_options boost_system boost_thread pthread)
##########################################################################
