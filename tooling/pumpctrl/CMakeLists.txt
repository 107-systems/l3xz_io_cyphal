##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(PUMPCTRL_TARGET pumpctrl)
##########################################################################
add_executable(${PUMPCTRL_TARGET}
    ../../src/phy/serial/AsyncSerial.cpp
    ../../src/driver/ssc32/SSC32.cpp
    ../../src/Pump.cpp
    pumpctrl.cpp
)
target_compile_options(${PUMPCTRL_TARGET} PRIVATE -std=c++17)
target_link_libraries(${PUMPCTRL_TARGET} boost_program_options boost_system boost_thread pthread)
##########################################################################
