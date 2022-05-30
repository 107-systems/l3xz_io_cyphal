/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/Walking.h>

#include <ros/console.h>

#include <gait/state/Standing.h>
#include <Util.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{
namespace
{

const float PITCH_MULT  = 0.75F;
const float FOOT_X      = +180.0F;
const float FOOT_Z_UP   = -100.0F;
const float FOOT_Z_DOWN = -200.0F;
const std::vector<KDL::Vector> FOOT_TRAJECTORY{
  {FOOT_X, +103.5 * PITCH_MULT, FOOT_Z_UP},
  {FOOT_X, +103.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, + 80.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, + 57.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, + 34.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, + 11.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, - 11.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, - 34.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, - 57.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, - 80.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, -103.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, -103.5 * PITCH_MULT, FOOT_Z_UP},
};

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Walking::onEnter()
{
  ROS_INFO("Walking ENTER");
}

void Walking::onExit()
{
  ROS_INFO("Walking EXIT");
}

std::tuple<StateBase *, ControllerOutput> Walking::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  const auto sample = [&](const std::uint8_t leg_index, const bool flip, const float y_offset = 0){
    auto pos = interpolatePiecewiseClosed(wrapPhase(_phase + (leg_index / 6.0F)),
                                          FOOT_TRAJECTORY.data(),
                                          FOOT_TRAJECTORY.size());
    pos[1] += y_offset;
    pos[1] *= flip ? -1.0F : +1.0F;
    return pos;
  };
  std::map<Leg, KDL::Vector> leg_pos;
  const float extension_front = 100;
  leg_pos[Leg::RightFront]  = sample(0, false, extension_front);
  leg_pos[Leg::LeftMiddle]  = sample(1, true);
  leg_pos[Leg::RightBack]   = sample(2, false);
  leg_pos[Leg::LeftFront]   = sample(3, true, extension_front);
  leg_pos[Leg::RightMiddle] = sample(4, false);
  leg_pos[Leg::LeftBack]    = sample(5, true);

  ControllerOutput next_output = prev_output;
  for (const auto& [leg, pos] : leg_pos)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);
    common::kinematic::IK_Input const ik_input(pos(0), pos(1), pos(2),
                                               coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
    auto const ik_output = engine.ik_solve(ik_input);
    if (!ik_output.has_value()) {
      ROS_ERROR("Walking::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
        pos(0), pos(1), pos(2), coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
      return {this, next_output};
    }
    next_output.set_angle_deg(leg, Joint::Coxa,  ik_output.value().angle_deg(Joint::Coxa));
    next_output.set_angle_deg(leg, Joint::Femur, ik_output.value().angle_deg(Joint::Femur));
    next_output.set_angle_deg(leg, Joint::Tibia, ik_output.value().angle_deg(Joint::Tibia));
    if (leg == Leg::LeftFront)
    {
      ROS_INFO("Walking::update Front/Left foot pos: %f %f %f", pos(0), pos(1), pos(2));
    }
  }

  _phase += _phase_increment;
  return std::tuple((std::abs(_phase) < 1.0F) ? this : static_cast<StateBase*>(new Standing), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
