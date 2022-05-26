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

#include <gait/state/Standing.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
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

std::tuple<StateBase *, GaitControllerOutput> TurningRight::update(common::kinematic::Engine const & engine, GaitControllerInput const & input, GaitControllerOutput const & prev_output)
{
  GaitControllerOutput next_output = prev_output;

  /* TODO: Walk one gait::state cycle Backward. */

  return std::tuple(new Standing(), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
