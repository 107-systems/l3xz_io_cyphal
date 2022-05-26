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

std::tuple<StateBase *, ControllerOutput> StandUp::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  // static double const TARGET_TIBIA_TIP_x =  175.0;
  // static double const TARGET_TIBIA_TIP_y =    0.0;
  // static double const TARGET_TIBIA_TIP_z = -200.0;

  // double x = TARGET_TIBIA_TIP_x,
  //        y = TARGET_TIBIA_TIP_y,
  //        z = TARGET_TIBIA_TIP_z;

  bool all_target_angles_reached = true;
  for (auto leg : LEG_LIST)
  {
    // /* Calculate required target angles for desired
    //  * target position and set the output actuators.
    //  */
    // double const coxa_deg_actual  = input.at(leg, Joint::Coxa );
    // double const femur_deg_actual = input.at(leg, Joint::Femur);
    // double const tibia_deg_actual = input.at(leg, Joint::Tibia);

    // common::kinematic::IK_Input const ik_input.at(x, y, z, coxa_deg_actual, femur_deg_actual, tibia_deg_actual);

    // auto const ik_output = engine.ik_solve(ik_input);

    // if (!ik_output.has_value()) {
    //   ROS_ERROR("StandUp::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
    //     x, y, z, coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
    //   return this;
    // }

    /* Set output to the angle actuators. */
    // output(leg, Joint::Coxa )->set(ik_output.value().angle_deg(Joint::Coxa));
    // output(leg, Joint::Femur)->set(ik_output.value().angle_deg(Joint::Femur));
    // output(leg, Joint::Tibia)->set(ik_output.value().angle_deg(Joint::Tibia));

    float const COXA_TARGET  =  0.0f;
    float const FEMUR_TARGET = -45.0f;
    float const TIBIA_TARGET = -45.0f;

    next_output.at(leg, Joint::Coxa)  = COXA_TARGET;
    next_output.at(leg, Joint::Femur) = FEMUR_TARGET;
    next_output.at(leg, Joint::Tibia) = TIBIA_TARGET;



    /* Check if target angles have been reached. */
    float const coxa_angle_actual = input.at(leg, Joint::Coxa);
    //float const coxa_angle_error = fabs(ik_output.value().angle_deg(Joint::Coxa) - coxa_angle_actual);
    float const coxa_angle_error = fabs(COXA_TARGET - coxa_angle_actual);
    bool  const coxa_is_initial_angle_reached = coxa_angle_error < 2.0f;

    if (!coxa_is_initial_angle_reached) {
      ROS_INFO("gait::state::StandUp::update: coxa target angle not reached");
      all_target_angles_reached = false;
    }
 
 
    float const femur_angle_actual = input.at(leg, Joint::Femur);
    //float const femur_angle_error = fabs(ik_output.value().angle_deg(Joint::Femur) - femur_angle_actual);
    float const femur_angle_error = fabs(FEMUR_TARGET - femur_angle_actual);
    bool  const femur_is_initial_angle_reached = femur_angle_error < 2.0f;

    if (!femur_is_initial_angle_reached) {
      ROS_INFO("gait::state::StandUp::update: femur target angle not reached");
      all_target_angles_reached = false;
    }

    float const tibia_angle_actual = input.at(leg, Joint::Tibia);
    //float const tibia_angle_error = fabs(ik_output.value().angle_deg(Joint::Tibia) - tibia_angle_actual);
    float const tibia_angle_error = fabs(TIBIA_TARGET - tibia_angle_actual);
    bool  const tibia_is_initial_angle_reached = tibia_angle_error < 2.0f;

    if (!tibia_is_initial_angle_reached) {
      ROS_INFO("gait::state::StandUp::update: tibia target angle not reached");
      all_target_angles_reached = false;
    }
  }

  if (!all_target_angles_reached)
    return std::tuple(this, next_output);

  return std::tuple(new Standing(), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
