/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz/CoxaController.h>

#include <ros/console.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

CoxaController::CoxaController(std::unique_ptr<dynamixel::DynamixelController> mx28_ctrl)
: _mx28_ctrl{std::move(mx28_ctrl)}
{
  if (auto [err, _mx28_id_vect] = _mx28_ctrl->broadcastPing(); err == dynamixel::Error::None)
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

void CoxaController::turnLedOn()
{
  uint8_t led_on = 1;
  _mx28_ctrl->syncWrite(65, sizeof(led_on), std::make_tuple(1, &led_on));
}

void CoxaController::turnLedOff()
{
  uint8_t led_off = 0;
  _mx28_ctrl->syncWrite(65, sizeof(led_off), std::make_tuple(1, &led_off));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
