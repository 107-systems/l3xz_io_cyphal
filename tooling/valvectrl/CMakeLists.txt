##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(VALVECTRL_TARGET valvectrl)
##########################################################################
add_executable(${VALVECTRL_TARGET}
    ../../src/phy/serial/AsyncSerial.cpp
    ../../src/driver/ssc32/SSC32.cpp
    ../../src/Valve.cpp
    valvectrl.cpp
)
target_compile_options(${VALVECTRL_TARGET} PRIVATE -std=c++17)
target_link_libraries(${VALVECTRL_TARGET} boost_program_options boost_system boost_thread pthread)
##########################################################################
