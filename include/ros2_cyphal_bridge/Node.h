/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_cyphal_bridge/graphs/contributors.
 */

#ifndef ROS_ROS_BRIDGE_NODE_H_
#define ROS_ROS_BRIDGE_NODE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "CanManager.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();


private:
  std::unique_ptr<CanManager> _can_mgr;

  rclcpp::TimerBase::SharedPtr _io_loop_timer;
  std::chrono::steady_clock::time_point _prev_io_loop_timepoint;
  static std::chrono::milliseconds constexpr IO_LOOP_RATE{10};
  void io_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* ROS_ROS_BRIDGE_NODE_H_ */
