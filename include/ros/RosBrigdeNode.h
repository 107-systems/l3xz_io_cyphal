/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef ROS_ROS_BRIDGE_NODE_H_
#define ROS_ROS_BRIDGE_NODE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int16.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class RosBridgeNode : public rclcpp::Node
{
public:
  RosBridgeNode();

  void publish_radiation_tick_count(int16_t const radiation_tick_cnt);

  inline TeleopCommandData teleop_cmd_data() const { return _teleop_cmd_data;  }

private:
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr _radiation_pub;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  TeleopCommandData _teleop_cmd_data;

  void onCmdVelUpdate(geometry_msgs::msg::Twist const & msg);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* ROS_ROS_BRIDGE_NODE_H_ */
