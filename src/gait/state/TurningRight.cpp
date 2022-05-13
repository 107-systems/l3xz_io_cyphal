/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/TurningRight.h>

#include <ros/console.h>

#include <gait/state/StandingState.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void TurningRight::onEnter()
{
  ROS_INFO("TurningRight ENTER");
}

void TurningRight::onExit()
{
  ROS_INFO("TurningRight EXIT");
}

GaitControllerState * TurningRight::update(GaitControllerInput const & input, GaitControllerOutput & output)
{
  /* TODO: Walk one gait cycle Backward. */
  return new StandingState();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */
