/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz/dynamixel/MX28Controller.h>

#include <ros/console.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixel
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

MX28Controller::MX28Controller(std::unique_ptr<DynamixelController> dyn_ctrl)
: _dyn_ctrl{std::move(dyn_ctrl)}
{
  if (auto [err, _mx28_id_vect] = _dyn_ctrl->broadcastPing(); err == Error::None)
  {
    ROS_INFO("Detected Dynamixel:");
    for (uint8_t id : _mx28_id_vect)
      ROS_INFO("[ID:%03d]", id);
  }
  else
    ROS_ERROR("%s::%s error, 'broadcastPing()' failed with %d", __FILE__, __FUNCTION__, static_cast<int>(err));
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void MX28Controller::turnLedOn()
{
  uint8_t led_on = 1;
  _dyn_ctrl->syncWrite(65, sizeof(led_on), std::make_tuple(1, &led_on));
}

void MX28Controller::turnLedOff()
{
  uint8_t led_off = 0;
  _dyn_ctrl->syncWrite(65, sizeof(led_off), std::make_tuple(1, &led_off));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixel */
