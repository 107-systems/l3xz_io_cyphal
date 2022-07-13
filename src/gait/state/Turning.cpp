/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/Turning.h>

#include <gait/state/Standing.h>
#include <gait/state/Walking.h>

namespace gait::state
{

void Turning::onEnter()
{
  printf("[INFO] Turning ENTER");
}

void Turning::onExit()
{
  printf("[INFO] Turning EXIT");
}

std::tuple<StateBase *, ControllerOutput> Turning::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;
  for (const auto leg : LEG_LIST)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);
    const auto lt = Walking::getLegTraits(leg);
    const auto pos = Walking::sampleFootTrajectory(lt, (lt.is_left == _left) ? +_phase : -_phase);
    common::kinematic::IK_Input const ik_input(pos(0), pos(1), pos(2),
                                               coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
    auto const ik_output = engine.ik_solve(ik_input);
    if (!ik_output.has_value()) {
      printf("[ERROR] Turning::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
        pos(0), pos(1), pos(2), coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
      return {this, next_output};
    }
    next_output.set_angle_deg(leg, Joint::Coxa,  ik_output.value().angle_deg(Joint::Coxa));
    next_output.set_angle_deg(leg, Joint::Femur, ik_output.value().angle_deg(Joint::Femur));
    next_output.set_angle_deg(leg, Joint::Tibia, ik_output.value().angle_deg(Joint::Tibia));
  }
  _phase += PHASE_INCREMENT;
  return std::tuple((_phase < 1.0F) ? this : static_cast<StateBase*>(new Standing), next_output);
}

} /* gait::state */
