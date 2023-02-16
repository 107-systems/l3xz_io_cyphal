/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef L3XZ_ROS_CYPHAL_BRIDGE_CANMANAGER_H
#define L3XZ_ROS_CYPHAL_BRIDGE_CANMANAGER_H

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <socketcan.h>

#include <string>
#include <thread>
#include <atomic>
#include <functional>

#include <rclcpp/rclcpp.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class CanManager
{
public:
  typedef std::function<void(CanardFrame const &)> OnCanFrameReceivedFunc;

  CanManager(rclcpp::Logger const logger,
             std::string const & iface_name,
             OnCanFrameReceivedFunc on_can_frame_received);
  ~CanManager();


  bool transmit(CanardFrame const & frame);


private:
  rclcpp::Logger const _logger;
  std::string const IFACE_NAME;
  bool const IS_CAN_FD;
  int _socket_can_fd;
  OnCanFrameReceivedFunc _on_can_frame_received;

  std::atomic<bool> _rx_thread_active;
  std::thread _rx_thread;
  void rx_thread_func();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* L3XZ_ROS_CYPHAL_BRIDGE_CANMANAGER_H */
