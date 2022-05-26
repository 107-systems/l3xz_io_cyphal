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

#include <gait/state/Standing.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

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

std::tuple<StateBase *, ControllerOutput> TurningLeft::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  /* TODO: Walk one gait::state cycle Backward. */

  return std::tuple(new Standing(), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
