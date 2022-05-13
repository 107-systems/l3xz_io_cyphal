/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/InitState.h>

#include <ros/console.h>

#include <gait/state/StandingState.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

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

StateBase * InitState::update(GaitControllerInput const & input, GaitControllerOutput & output)
{
  /* TODO: Drive to initial position. */
  return new StandingState();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
