/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/Standing.h>

#include <ros/console.h>

#include <gait/state/TurningLeft.h>
#include <gait/state/TurningRight.h>
#include <gait/state/ForwardWalking.h>
#include <gait/state/BackwardWalking.h>

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
  ROS_INFO("Standing ENTER");
}

void Standing::onExit()
{
  ROS_INFO("Standing EXIT");
}

std::tuple<StateBase *, ControllerOutput> Standing::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  if      (input.teleop_cmd().linear_velocity_x > 0.2f)
    return std::tuple(new ForwardWalking(), next_output);
  else if (input.teleop_cmd().linear_velocity_x < -0.2f)
    return std::tuple(new BackwardWalking(), next_output);
  else if (input.teleop_cmd().angular_velocity_z > 0.2f)
    return std::tuple(new TurningRight(), next_output);
  else if (input.teleop_cmd().angular_velocity_z < -0.2f)
    return std::tuple(new TurningLeft(), next_output);
  else
    return std::tuple(this, next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
