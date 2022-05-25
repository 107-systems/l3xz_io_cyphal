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
    output.at(leg, Joint::Coxa ) = INITIAL_COXA_ANGLE_DEG;
    output.at(leg, Joint::Femur) = INITIAL_FEMUR_ANGLE_DEG;
    output.at(leg, Joint::Tibia) = INITIAL_TIBIA_ANGLE_DEG;
  }

  /* Check if target angles have been reached. */
  bool all_target_angles_reached = true;
  for (auto leg : LEG_LIST)
  {
    float const coxa_angle_actual = input(leg, Joint::Coxa)->get().value();
    float const coxa_angle_error = fabs(INITIAL_COXA_ANGLE_DEG - coxa_angle_actual);
    bool  const coxa_is_initial_angle_reached = coxa_angle_error < 5.0f;

    if (!coxa_is_initial_angle_reached) {
      ROS_INFO("gait::state::Init::update: coxa target angle not reached for %s", input(leg, Joint::Coxa)->name().c_str());
      all_target_angles_reached = false;
    }
 
 
    float const femur_angle_actual = input(leg, Joint::Femur)->get().value();
    float const femur_angle_error = fabs(INITIAL_FEMUR_ANGLE_DEG - femur_angle_actual);
    bool  const femur_is_initial_angle_reached = femur_angle_error < 5.0f;

    if (!femur_is_initial_angle_reached) {
      ROS_INFO("gait::state::Init::update: femur target angle not reached for %s", input(leg, Joint::Femur)->name().c_str());
      all_target_angles_reached = false;
    }

    float const tibia_angle_actual = input(leg, Joint::Tibia)->get().value();
    float const tibia_angle_error = fabs(INITIAL_TIBIA_ANGLE_DEG - tibia_angle_actual);
    bool  const tibia_is_initial_angle_reached = tibia_angle_error < 5.0f;

    if (!tibia_is_initial_angle_reached) {
      ROS_INFO("gait::state::Init::update: tibia target angle not reached for %s", input(leg, Joint::Tibia)->name().c_str());
      all_target_angles_reached = false;
    }
  }

  if (!all_target_angles_reached)
    return this;

  /* All good, let's transition to the next state. */
  return this;//new StandUp();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
