/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <head/state/Teleop.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Teleop::onEnter()
{

}

void Teleop::onExit()
{

}

std::tuple<StateBase *, ControllerOutput> Teleop::update(ControllerInput const & input, ControllerOutput const & prev_output)
{
  /* Calculate new values for sensor head, both pan and tilt joint
   * based on the input provided by the teleop node.
   */
  static float const MAX_ANGLE_INCREMENT_PER_CYCLE_DEG = 10.0f;

  float const pan_angle_actual = input._angle_sensor_sensor_head_pan->get().value();
  float const pan_angle_target = pan_angle_actual + (input._teleop_cmd.angular_velocity_head_pan * MAX_ANGLE_INCREMENT_PER_CYCLE_DEG);

  float const tilt_angle_actual = input._angle_sensor_sensor_head_tilt->get().value();
  float const tilt_angle_target = tilt_angle_actual + (input._teleop_cmd.angular_velocity_head_tilt * MAX_ANGLE_INCREMENT_PER_CYCLE_DEG);

  return std::tuple(this, ControllerOutput(pan_angle_target, tilt_angle_target));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head::state */
