/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <ros/RosBrigdeNode.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

RosBridgeNode::RosBridgeNode()
: Node("l3xz")
, _teleop_cmd_data{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
{
  _radiation_pub = create_publisher<std_msgs::msg::Int16>("/l3xz/radiation_tick_cnt", 25);

  _cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel", 10, [this](geometry_msgs::msg::Twist const & msg) { this->onCmdVelUpdate(msg); });
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void RosBridgeNode::publish_radiation_tick_count(int16_t const radiation_tick_cnt)
{
  std_msgs::msg::Int16 radiation_tick_msg;
  radiation_tick_msg.data = radiation_tick_cnt;
  _radiation_pub->publish(radiation_tick_msg);
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void RosBridgeNode::onCmdVelUpdate(geometry_msgs::msg::Twist const & msg)
{
  _teleop_cmd_data.linear_velocity_x           = msg.linear.x;
  _teleop_cmd_data.linear_velocity_y           = msg.linear.y;
  _teleop_cmd_data.angular_velocity_head_tilt  = msg.angular.x;
  _teleop_cmd_data.angular_velocity_head_pan   = msg.angular.y;
  _teleop_cmd_data.angular_velocity_z          = msg.angular.z;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
