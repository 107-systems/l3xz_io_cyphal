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

#include <mutex>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <cyphal++/cyphal++.h>

#include <std_msgs/msg/float32.hpp>

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
  ~Node();


private:
  std::unique_ptr<CanManager> _can_mgr;

  static size_t constexpr CYPHAL_O1HEAP_SIZE = (::Node::DEFAULT_O1HEAP_SIZE * 16);
  static size_t constexpr CYPHAL_TX_QUEUE_SIZE = 256;
  static size_t constexpr CYPHAL_RX_QUEUE_SIZE = 256;
  ::Node::Heap<CYPHAL_O1HEAP_SIZE> _node_heap;
  ::Node _node_hdl;
  std::mutex _node_mtx;

  std::map<CanardPortID, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> _angle_actual_ros_pub;
  std::map<CanardPortID, ::Subscription> _angle_actual_cyphal_sub;
  void init_cyphal_to_ros();

  CanardMicrosecond micros();

  std::chrono::steady_clock::time_point _prev_io_loop_timepoint;
  static std::chrono::milliseconds constexpr IO_LOOP_RATE{1};
  rclcpp::TimerBase::SharedPtr _io_loop_timer;
  void io_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* ROS_ROS_BRIDGE_NODE_H_ */
