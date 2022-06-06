/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <driver/orel20/Orel20.h>

#include <stdexcept>

#include <ros/ros.h>
#include <ros/console.h>

/**************************************************************************************
 * EXTERN DECLARATION
 **************************************************************************************/

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace driver
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Orel20::Orel20(uint8_t const dronecan_node_id)
: _node{getCanDriver(), getSystemClock()}
, _esc_pub{_node}
, _rpm_val{0}
{
  _node.setNodeID(dronecan_node_id);
  _node.setName("driver.orel20");

  if (auto const rc = _node.start(); rc < 0)
    throw std::runtime_error("Orel20::Orel20: failed to start the node, error: " + std::to_string(rc));

  if (auto const rc = _esc_pub.init(); rc < 0)
    throw std::runtime_error("Orel20::Orel20: failed to start the publisher, error: " + std::to_string(rc));

  _node.setModeOperational();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Orel20::spinOnce()
{
  uavcan::equipment::esc::RPMCommand rpm_cmd;
  rpm_cmd.rpm.push_back(_rpm_val);

  if (auto const rc = _esc_pub.broadcast(rpm_cmd); rc < 0)
    ROS_ERROR("Orel20::loop: ESC RPMCommand message publication failure: %d", rc);

  if (auto const rc = _node.spinOnce(); rc < 0)
    ROS_WARN("Orel20::loop: transient failure: %d", rc);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* driver */
