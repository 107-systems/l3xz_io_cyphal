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

#include <types/LegJointKey.h>

#include <common/actuator/interface/AnglePositionActuator.h>

#include <phy/opencyphal/Node.hpp>
#include <phy/opencyphal/SocketCAN.h>

#include <driver/ssc32/SSC32.h>
#include <driver/orel20/Orel20.h>

#include <glue/l3xz/ELROB2022/SSC32PWMActuatorBulkwriter.h>
#include <glue/BumperSensorReader.h>
#include <glue/HydraulicAnglePositionReader.h>
#include <glue/DynamixelAnglePositionWriter.h>

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
  IoNode(
    dynamixel::SharedMX28 mx28_ctrl,
    driver::SharedSSC32 ssc32_ctrl,
    glue::l3xz::ELROB2022::SSC32PWMActuatorBulkwriter & ssc32_pwm_actuator_bulk_driver
  );

private:
  phy::opencyphal::SocketCAN _open_cyphal_can_if;
  phy::opencyphal::Node _open_cyphal_node;
  dynamixel::SharedMX28 _mx28_ctrl;
  driver::Orel20 _hydraulic_pump;

  glue::BumperSensorReader _bumber_sensor_reader;
  glue::HydraulicAnglePositionReader _hydraulic_angle_position_reader;
  glue::l3xz::ELROB2022::SSC32PWMActuatorBulkwriter & _ssc32_pwm_actuator_bulk_driver;
  glue::DynamixelAnglePositionWriter _dynamixel_angle_position_writer;

  rclcpp::TimerBase::SharedPtr _timer;

  rclcpp::Publisher<l3xz_gait_ctrl::msg::LegAngle>::SharedPtr _leg_angle_pub;
  rclcpp::Subscription<l3xz_gait_ctrl::msg::LegAngle>::SharedPtr _leg_angle_sub;
  l3xz_gait_ctrl::msg::LegAngle _leg_angle_target_msg;

  rclcpp::Publisher<l3xz_head_ctrl::msg::HeadAngle>::SharedPtr _head_angle_pub;
  rclcpp::Subscription<l3xz_head_ctrl::msg::HeadAngle>::SharedPtr _head_angle_sub;
  l3xz_head_ctrl::msg::HeadAngle _head_angle_target_msg;

  void timerCallback();

  static float get_angle_deg(l3xz_gait_ctrl::msg::LegAngle const & msg, Leg const leg, Joint const joint);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* ROS_ROS_BRIDGE_NODE_H_ */
