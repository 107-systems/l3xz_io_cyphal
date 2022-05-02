/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <state/TurningLeft.h>

#include <ros/console.h>

#include <state/StandingState.h>

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

RobotState * TurningLeft::update(TeleopCommandData const cmd, RobotStateInput & input, RobotStateOutput & output)
{
  /* TODO: Walk one gait cycle Backward. */
  return new StandingState();
}
