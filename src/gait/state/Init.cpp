/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/Init.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <gait/state/StandUp.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Init::onEnter()
{
  ROS_INFO("Init ENTER");
}

void Init::onExit()
{
  ROS_INFO("Init EXIT");
}

StateBase * Init::update(common::kinematic::Engine const & engine, GaitControllerInput & input, GaitControllerOutput & output)
{
  /* Set the desired target angle. */
  for (auto leg : LEG_LIST)
  {
    output(leg, Joint::Coxa )->set(INITIAL_COXA_ANGLE);
    output(leg, Joint::Femur)->set(INITIAL_FEMUR_ANGLE);
    output(leg, Joint::Tibia)->set(INITIAL_TIBIA_ANGLE);
  }

  /* Check if we have valid angles. */
  for (auto [leg, joint] : LEG_JOINT_LIST)
  {
    if (!input(leg, joint)->get().has_value()) {
      ROS_ERROR("gait::state::Init::update: no valid input data for %s", input(leg, joint)->name().c_str());
      return this;
    }
  }

  /* Check if we have reached the desired target angle. */
  static std::list<GaitControllerInput::AngleSensorMapKey> const COXA_ANGLE_SENSOR_KEY_LIST =
  {
    std::tuple(Leg::LeftFront,   Joint::Coxa),
    std::tuple(Leg::RightFront,  Joint::Coxa),
    std::tuple(Leg::LeftMiddle,  Joint::Coxa),
    std::tuple(Leg::RightMiddle, Joint::Coxa),
    std::tuple(Leg::LeftMiddle,  Joint::Coxa),
    std::tuple(Leg::RightMiddle, Joint::Coxa),
  };

  bool all_target_angles_reached = true;
  for (auto [leg, joint] : COXA_ANGLE_SENSOR_KEY_LIST)
  {
    float const coxa_angle_actual = input(leg, joint)->get().value();
    float const coxa_angle_diff = fabs(INITIAL_COXA_ANGLE - coxa_angle_actual);
    bool  const coxa_is_initial_angle_reached = coxa_angle_diff < 1.0f;

    if (!coxa_is_initial_angle_reached) {
      ROS_INFO("gait::state::Init::update: target angle not reached for %s", input(leg, joint)->name().c_str());
      all_target_angles_reached = false;
    }
  }

  if (!all_target_angles_reached)
    return this;

  /* All good, let's transition to the next state. */
  return new StandUp();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
