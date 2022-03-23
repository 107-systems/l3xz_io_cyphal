##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(OROCOS_TARGET orocos)
##########################################################################
find_package(Eigen3)
##########################################################################
add_executable(${OROCOS_TARGET}
  orocos.cpp
)
target_include_directories(${OROCOS_TARGET} PRIVATE ${EIGEN3_INCLUDE_DIR})
target_compile_options(${OROCOS_TARGET} PRIVATE -std=c++17)
target_link_libraries(${OROCOS_TARGET} orocos-kdl)
##########################################################################
