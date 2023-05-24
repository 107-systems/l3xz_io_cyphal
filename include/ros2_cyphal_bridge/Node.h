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

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <ros2_heartbeat/Publisher.h>

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

  std::chrono::steady_clock::time_point const _node_start;

  static std::chrono::milliseconds constexpr HEARTBEAT_LOOP_RATE{100};
  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  ::Publisher<uavcan::node::Heartbeat_1_0> _cyphal_heartbeat_pub;
  std::chrono::steady_clock::time_point _prev_heartbeat_timepoint;
  static std::chrono::milliseconds constexpr CYPHAL_HEARTBEAT_PERIOD{1000};
  void init_cyphal_heartbeat();

  ::NodeInfo _cyphal_node_info;
  void init_cyphal_node_info();

  std::map<CanardPortID, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> _angle_actual_ros_pub;
  std::map<CanardPortID, ::Subscription> _angle_actual_cyphal_sub;
  void init_cyphal_to_ros_angle_actual();

  std::map<CanardPortID, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> _tibia_endpoint_switch_ros_pub;
  std::map<CanardPortID, ::Subscription> _tibia_endpoint_switch_cyphal_sub;
  void init_cyphal_to_ros_tibia_endpoint_switch();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _estop_ros_pub;
  ::Subscription _estop_cyphal_sub;
  void init_cyphal_to_ros_estop();

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr _radiation_tick_cnt_ros_pub;
  ::Subscription _radiation_tick_cnt_cyphal_sub;
  void init_cyphal_to_ros_radiation_tick_cnt();

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _light_mode_ros_sub;
  ::Publisher<uavcan::primitive::scalar::Integer8_1_0> _light_mode_cyphal_pub;
  void init_ros_to_cyphal_light_mode();

  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr _servo_pulse_width_ros_sub;
  ::Publisher<uavcan::primitive::array::Natural16_1_0> _servo_pulse_width_cyphal_pub;
  void init_ros_to_cyphal_servo_pulse_width();

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _pump_readiness_ros_sub;
  ::Publisher<reg::udral::service::common::Readiness_0_1> _pump_readiness_cyphal_pub;
  void init_ros_to_cyphal_pump_readiness();

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _pump_rpm_setpoint_ros_sub;
  ::Publisher<reg::udral::service::actuator::common::sp::Scalar_0_1> _pump_rpm_setpoint_cyphal_pub;
  void init_ros_to_cyphal_pump_setpoint();

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
