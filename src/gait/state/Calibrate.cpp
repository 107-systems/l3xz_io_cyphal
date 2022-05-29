/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/Calibrate.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <gait/state/Init.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Calibrate::Calibrate(driver::SharedSSC32 ssc32_ctrl, driver::SharedOrel20 orel20_ctrl)
: _ssc32_ctrl{ssc32_ctrl}
, _orel20_ctrl{orel20_ctrl}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Calibrate::onEnter()
{
  ROS_INFO("Calibrate ENTER");

  /* Open all hydraulic valves. */
  for (auto ch: SERVO_CHANNEL_LIST)
      _ssc32_ctrl->setPulseWidth(ch, 2000, 50);

  /* Start the hydraulic pump. */
  _orel20_ctrl->setRPM(10);
}

void Calibrate::onExit()
{
  /* Close all hydraulic valves. */
  for (auto ch: SERVO_CHANNEL_LIST)
    _ssc32_ctrl->setPulseWidth(ch, 1500, 50);

  /* Stop the hydraulic pump. */
  _orel20_ctrl->setRPM(0);

  ROS_INFO("Calibrate EXIT");
}

std::tuple<StateBase *, ControllerOutput> Calibrate::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  return std::tuple(this, prev_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
