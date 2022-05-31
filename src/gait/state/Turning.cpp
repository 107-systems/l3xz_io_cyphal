/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/Turning.h>

#include <ros/console.h>

#include <gait/state/Standing.h>
#include <gait/state/Walking.h>

namespace gait::state
{

void Turning::onEnter()
{
  ROS_INFO("Turning ENTER");
}

void Turning::onExit()
{
  ROS_INFO("Turning EXIT");
}

std::tuple<StateBase *, ControllerOutput> Turning::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  /* TODO: Walk one gait::state cycle Backward. */

  return std::tuple(new Standing(), next_output);
}

} /* gait::state */
