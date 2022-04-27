/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <state/ForwardWalking.h>

#include <ros/console.h>

#include <state/StandingState.h>

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ForwardWalking::onEnter()
{
  ROS_INFO("ForwardWalking ENTER");
}

void ForwardWalking::onExit()
{
  ROS_INFO("ForwardWalking EXIT");
}

RobotState * ForwardWalking::update(l3xz::TeleopCommandData const cmd, RobotStateInput & input, RobotStateOutput & output)
{
  /* TODO: Walk one gait cycle forward. */
  return new StandingState();
}
