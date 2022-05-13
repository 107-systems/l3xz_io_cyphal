/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/TurningLeft.h>

#include <ros/console.h>

#include <gait/state/StandingState.h>

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void TurningLeft::onEnter()
{
  ROS_INFO("TurningLeft ENTER");
}

void TurningLeft::onExit()
{
  ROS_INFO("TurningLeft EXIT");
}

GaitControllerState * TurningLeft::update(GaitControllerInput const & input, GaitControllerOutput & output)
{
  /* TODO: Walk one gait cycle Backward. */
  return new StandingState();
}
