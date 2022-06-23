/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/Standing.h>

#include <gait/state/Turning.h>
#include <gait/state/Walking.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Standing::onEnter()
{
  printf("[INFO] Standing ENTER");
}

void Standing::onExit()
{
  printf("[INFO] Standing EXIT");
}

std::tuple<StateBase *, ControllerOutput> Standing::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;
  if (std::abs(input.teleop_cmd().linear_velocity_x) > 0.2f)
  {
    return std::tuple(new Walking(input.teleop_cmd().linear_velocity_x > 0), next_output);
  }
  if (std::abs(input.teleop_cmd().angular_velocity_z) > 0.2f)
  {
    return std::tuple(new Turning(input.teleop_cmd().angular_velocity_z > 0), next_output);
  }
  return std::tuple(this, next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
