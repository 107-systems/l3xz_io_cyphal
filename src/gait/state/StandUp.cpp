/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/StandUp.h>

#include <ros/ros.h>
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

void StandUp::onEnter()
{
  ROS_INFO("StandUp ENTER");
}

void StandUp::onExit()
{
  ROS_INFO("StandUp EXIT");
}

StateBase * StandUp::update(common::kinematic::Engine const & engine, GaitControllerInput & input, GaitControllerOutput & output)
{
  static double const TARGET_TIBIA_TIP_x =  175.0;
  static double const TARGET_TIBIA_TIP_y =    0.0;
  static double const TARGET_TIBIA_TIP_z = -150.0;

  double x = TARGET_TIBIA_TIP_x,
         y = TARGET_TIBIA_TIP_y,
         z = TARGET_TIBIA_TIP_z;

  for (auto leg : LEG_LIST)
  {
    /* Calculate required target angles for desired
     * target position and set the output actuators.
     */
    double const coxa_deg_actual  = input(leg, Joint::Coxa )->get().value();
    double const femur_deg_actual = input(leg, Joint::Femur)->get().value();
    double const tibia_deg_actual = input(leg, Joint::Tibia)->get().value();

    common::kinematic::IK_Input const ik_input(x, y, z, coxa_deg_actual, femur_deg_actual, tibia_deg_actual);

    auto const ik_output = engine.ik_solve(ik_input);

    if (!ik_output.has_value()) {
      ROS_ERROR("StandUp::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
        x, y, z, coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
      return this;
    }

    /* Set output to the angle actuators.
     */
    output(leg, Joint::Coxa )->set(ik_output.value().angle_deg(Joint::Coxa));
    output(leg, Joint::Femur)->set(ik_output.value().angle_deg(Joint::Femur));
    output(leg, Joint::Tibia)->set(ik_output.value().angle_deg(Joint::Tibia));
  }

  return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
