/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <state/TurningRight.h>

#include <ros/console.h>

#include <state/StandingState.h>

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

GaitControllerState * TurningRight::update(GaitControllerStateInput const & input, GaitControllerStateOutput & output)
{
  /* TODO: Walk one gait cycle Backward. */
  return new StandingState();
}
