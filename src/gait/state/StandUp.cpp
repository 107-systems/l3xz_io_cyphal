/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/StandUp.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <gait/state/Standing.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void StandUp::onEnter()
{
  ROS_INFO("StandUp ENTER");
}

void StandUp::onExit()
{
  ROS_INFO("StandUp EXIT");
}

StateBase * StandUp::update(GaitControllerInput & input, GaitControllerOutput & output)
{
  return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
