##########################################################################
cmake_minimum_required(VERSION 3.16)
##########################################################################
set(LIBSWEEP_TARGET "sweep")
##########################################################################
set(SWEEP_VERSION_MAJOR 1)
set(SWEEP_VERSION_MINOR 3)
set(SWEEP_VERSION_PATCH 0)
##########################################################################
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/include/sweep/config.h)
##########################################################################
add_library(${LIBSWEEP_TARGET} STATIC "")
##########################################################################
target_include_directories(${LIBSWEEP_TARGET} PUBLIC include include/sweep ${CMAKE_CURRENT_BINARY_DIR}/include)
##########################################################################
target_sources(${LIBSWEEP_TARGET}
  PRIVATE
  src/unix/serial.cc
  src/protocol.cc
  src/sweep.cc
)
##########################################################################
target_compile_options(${LIBSWEEP_TARGET} PRIVATE -std=c++11 -Wall -Wextra -pedantic -fvisibility=hidden -fno-rtti)
##########################################################################
target_link_libraries(${LIBSWEEP_TARGET} pthread)
##########################################################################
