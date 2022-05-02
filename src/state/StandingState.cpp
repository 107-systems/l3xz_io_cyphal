/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <state/StandingState.h>

#include <ros/console.h>

#include <state/ForwardWalking.h>
#include <state/BackwardWalking.h>

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void StandingState::onEnter()
{
  ROS_INFO("StandingState ENTER");
}

void StandingState::onExit()
{
  ROS_INFO("StandingState EXIT");
}

RobotState * StandingState::update(TeleopCommandData const cmd, RobotStateInput & input, RobotStateOutput & output)
{
  if (cmd.linear_velocity_x > 0.2)
    return new ForwardWalking();
  else if (cmd.linear_velocity_x < -0.2)
    return new BackwardWalking();
  else
    return this;
}
