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

StateBase * Standing::update(common::kinematic::Engine const & engine, GaitControllerInput & input, GaitControllerOutput & output)
{
  if      (input._teleop_cmd.linear_velocity_x > 0.2f)
    return new ForwardWalking();
  else if (input._teleop_cmd.linear_velocity_x < -0.2f)
    return new BackwardWalking();
  else if (input._teleop_cmd.angular_velocity_z > 0.2f)
    return new TurningRight();
  else if (input._teleop_cmd.angular_velocity_z < -0.2f)
    return new TurningLeft();
  else
    return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
