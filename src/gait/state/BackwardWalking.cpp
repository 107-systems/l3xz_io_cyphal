/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/BackwardWalking.h>

#include <ros/console.h>

#include <gait/state/StandingState.h>

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void BackwardWalking::onEnter()
{
  ROS_INFO("BackwardWalking ENTER");
}

void BackwardWalking::onExit()
{
  ROS_INFO("BackwardWalking EXIT");
}

GaitControllerState * BackwardWalking::update(GaitControllerInput const & input, GaitControllerOutput & output)
{
  /* TODO: Walk one gait cycle Backward. */
  return new StandingState();
}
