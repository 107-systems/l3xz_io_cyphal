/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <state/InitState.h>

#include <ros/console.h>

#include <state/StandingState.h>

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void InitState::onEnter()
{
  ROS_INFO("InitState ENTER");
}

void InitState::onExit()
{
  ROS_INFO("InitState EXIT");
}

RobotState * InitState::update(RobotStateInput const & input, RobotStateOutput & output)
{
  /* TODO: Drive to initial position. */
  return new StandingState();
}
