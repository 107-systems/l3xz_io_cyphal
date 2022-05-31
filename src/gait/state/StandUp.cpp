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
#include <gait/state/Walking.h>

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

  bool all_target_angles_reached = true;
  for (auto leg : LEG_LIST)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);

    /* Calculate required target angles for desired
     * target position and set the output actuators.
     */
    const auto pos = Walking::sampleFootTrajectory(Walking::getLegTraits(leg), 0);
    common::kinematic::IK_Input const ik_input(pos(0), pos(1), pos(2),
                                               coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
    auto const ik_output = engine.ik_solve(ik_input);

    if (!ik_output.has_value()) {
      ROS_ERROR("StandUp::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
        pos(0), pos(1), pos(2), coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
      return {this, next_output};
    }

    /* Set output to the angle actuators. */
    next_output.set_angle_deg(leg, Joint::Coxa,  ik_output.value().angle_deg(Joint::Coxa));
    next_output.set_angle_deg(leg, Joint::Femur, ik_output.value().angle_deg(Joint::Femur));
    next_output.set_angle_deg(leg, Joint::Tibia, ik_output.value().angle_deg(Joint::Tibia));

    /* Check if target angles have been reached. */
    float const coxa_angle_actual = input.get_angle_deg(leg, Joint::Coxa);
    float const coxa_angle_error = fabs(ik_output.value().angle_deg(Joint::Coxa) - coxa_angle_actual);
    bool  const coxa_is_initial_angle_reached = coxa_angle_error < 2.0f;

    if (!coxa_is_initial_angle_reached) {
      ROS_INFO("gait::state::StandUp::update: coxa target angle not reached");
      all_target_angles_reached = false;
    }
 
 
    float const femur_angle_actual = input.get_angle_deg(leg, Joint::Femur);
    float const femur_angle_error = fabs(ik_output.value().angle_deg(Joint::Femur) - femur_angle_actual);
    bool  const femur_is_initial_angle_reached = femur_angle_error < 2.0f;

    if (!femur_is_initial_angle_reached) {
      ROS_INFO("gait::state::StandUp::update: femur target angle not reached");
      all_target_angles_reached = false;
    }

    float const tibia_angle_actual = input.get_angle_deg(leg, Joint::Tibia);
    float const tibia_angle_error = fabs(ik_output.value().angle_deg(Joint::Tibia) - tibia_angle_actual);
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
