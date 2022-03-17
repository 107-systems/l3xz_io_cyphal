##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(SSC32CTRL_TARGET ssc32ctrl)
##########################################################################
add_executable(${SSC32CTRL_TARGET}
    ../../src/phy/serial/AsyncSerial.cpp
    ../../src/ssc32/SSC32.cpp
    ssc32ctrl.cpp
)
target_compile_options(${SSC32CTRL_TARGET} PRIVATE -std=c++17)
target_link_libraries(${SSC32CTRL_TARGET} boost_program_options boost_system boost_thread pthread)
##########################################################################
