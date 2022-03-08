##########################################################################
cmake_minimum_required(VERSION 3.16)
##########################################################################
set(DYNAMIXEL_SDK_TARGET "dynamixel")
##########################################################################
add_library(${DYNAMIXEL_SDK_TARGET} STATIC "")
##########################################################################
target_include_directories(${DYNAMIXEL_SDK_TARGET} PUBLIC current/include)
##########################################################################
target_sources(${DYNAMIXEL_SDK_TARGET}
  PRIVATE
  current/src/packet_handler.cpp
  current/src/protocol1_packet_handler.cpp
  current/src/protocol2_packet_handler.cpp
  current/src/group_sync_read.cpp
  current/src/group_sync_write.cpp
  current/src/group_bulk_read.cpp
  current/src/group_bulk_write.cpp
  current/src/port_handler.cpp
  current/src/port_handler_linux.cpp
)
##########################################################################
target_compile_options(${DYNAMIXEL_SDK_TARGET} PRIVATE -std=c++11)
##########################################################################
