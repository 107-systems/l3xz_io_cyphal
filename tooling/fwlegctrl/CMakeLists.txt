##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(FWLEGCTRL_TARGET fwlegctrl)
##########################################################################
add_executable(${FWLEGCTRL_TARGET}
    ../../src/phy/opencyphal/Node.cpp
    ../../src/phy/opencyphal/SocketCAN.cpp
    fwlegctrl.cpp
)
target_compile_options(${FWLEGCTRL_TARGET} PRIVATE -std=c++17 -Wall -Werror -pedantic)
target_link_libraries(${FWLEGCTRL_TARGET} pthread canard o1heap socketcan)
##########################################################################
