/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <head/state/Init.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <Const.h>

#include <head/state/Teleop.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Init::onEnter()
{

}

void Init::onExit()
{

}

std::tuple<StateBase *, ControllerOutput> Init::update(ControllerInput const & input, ControllerOutput const & prev_output)
{
  /* Check if we have reached the initial tilt angle. */
  float const tilt_angle_actual = input._angle_sensor_sensor_head_tilt->get().value();
  float const tilt_angle_error = fabs(INITIAL_TILT_ANGLE_DEG - tilt_angle_actual);
  bool  const tilt_is_initial_angle_reached = tilt_angle_error < INITIAL_ANGLE_EPSILON;

  /* Check if we have reached the initial pan angle. */
  float const pan_angle_actual = input._angle_sensor_sensor_head_pan->get().value();
  float const pan_angle_error = fabs(INITIAL_PAN_ANGLE_DEG - pan_angle_actual);
  bool  const pan_is_initial_angle_reached = pan_angle_error < INITIAL_ANGLE_EPSILON;

  /* If we have reached the initial angles transition
   * into the active state of the header controller.
   */
  if (tilt_is_initial_angle_reached && pan_is_initial_angle_reached)
    return std::tuple(new Teleop(), prev_output);

  return std::tuple(this, ControllerOutput(INITIAL_PAN_ANGLE_DEG, INITIAL_TILT_ANGLE_DEG));
}
 
/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head::state */
