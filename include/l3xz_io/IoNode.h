/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef ROS_ROS_BRIDGE_NODE_H_
#define ROS_ROS_BRIDGE_NODE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <l3xz_io/types/LegJointKey.h>

#include <l3xz_io/phy/opencyphal/Node.hpp>
#include <l3xz_io/phy/opencyphal/SocketCAN.h>

#include <l3xz_io/driver/orel20/Orel20.h>

#include <l3xz_io/glue/LegController.h>
#include <l3xz_io/glue/ValveController.h>
#include <l3xz_io/glue/OpenCyphalHeartbeatMonitor.h>
#include <l3xz_io/glue/DynamixelAnglePositionWriter.h>

#include <l3xz_gait_ctrl/msg/leg_angle.hpp>
#include <l3xz_head_ctrl/msg/head_angle.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class IoNode : public rclcpp::Node
{
public:
  IoNode();


private:
  enum class State
  {
    Init_Dynamixel,
    Init_LegController,
    Calibrate,
    Active
  };
  State _state;

  phy::opencyphal::SocketCAN _open_cyphal_can_if;
  phy::opencyphal::Node _open_cyphal_node;
  dynamixel::SharedDynamixel _dynamixel_ctrl;
  dynamixel::SharedMX28 _mx28_ctrl;
  driver::Orel20 _hydraulic_pump;

  glue::OpenCyphalHeartbeatMonitor _open_cyphal_heartbeat_monitor;
  glue::DynamixelAnglePositionWriter _dynamixel_angle_position_writer;
  glue::ValveController _valve_ctrl;
  glue::LegController _leg_ctrl;

  rclcpp::TimerBase::SharedPtr _timer;

  rclcpp::Publisher<l3xz_gait_ctrl::msg::LegAngle>::SharedPtr _leg_angle_pub;
  rclcpp::Subscription<l3xz_gait_ctrl::msg::LegAngle>::SharedPtr _leg_angle_sub;
  l3xz_gait_ctrl::msg::LegAngle _leg_angle_target_msg;

  rclcpp::Publisher<l3xz_head_ctrl::msg::HeadAngle>::SharedPtr _head_angle_pub;
  rclcpp::Subscription<l3xz_head_ctrl::msg::HeadAngle>::SharedPtr _head_angle_sub;
  l3xz_head_ctrl::msg::HeadAngle _head_angle_target_msg;

  void timerCallback();

  State handle_Init_Dynamixel();
  State handle_Init_LegController();
  State handle_Calibrate();
  State handle_Active();

  static float get_angle_deg(l3xz_gait_ctrl::msg::LegAngle const & msg, Leg const leg, Joint const joint);

  bool init_dynamixel();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* ROS_ROS_BRIDGE_NODE_H_ */
