/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <head/state/Init.h>

#include <ros/console.h>

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

StateBase * Init::update(HeadControllerInput const & input, HeadControllerOutput & output)
{
  /* The desired output in this state is always
   * the pre-configured initial angle for pan
   * and tilt element of the head.
   */
  output._angle_actuator_sensor_head_pan->set(INITIAL_PAN_ANGLE);
  output._angle_actuator_sensor_head_tilt->set(INITIAL_TILT_ANGLE);

  /* No state transition if we do not even have
   * valid input data.
   */
  if (!input._angle_sensor_sensor_head_pan->get().has_value()) {
    ROS_ERROR("head::state::Init::update: no valid input data for %s", input._angle_sensor_sensor_head_pan->name().c_str());
    return this;
  }

  if (!input._angle_sensor_sensor_head_tilt->get().value()) {
    ROS_ERROR("head::state::Init::update: no valid input data for %s", input._angle_sensor_sensor_head_tilt->name().c_str());
    return this;
  }


  /* Check if we have reached the initial tilt angle. */
  float const tilt_angle_actual = input._angle_sensor_sensor_head_tilt->get().value();
  float const tilt_angle_error = fabs(INITIAL_TILT_ANGLE - tilt_angle_actual);
  bool  const tilt_is_initial_angle_reached = tilt_angle_error < INITIAL_ANGLE_EPSILON;

  /* Check if we have reached the initial pan angle. */
  float const pan_angle_actual = input._angle_sensor_sensor_head_pan->get().value();
  float const pan_angle_error = fabs(INITIAL_PAN_ANGLE - pan_angle_actual);
  bool  const pan_is_initial_angle_reached = pan_angle_error < INITIAL_ANGLE_EPSILON;

  /* If we have reached the initial angles transition
   * into the active state of the header controller.
   */
  if (tilt_is_initial_angle_reached && pan_is_initial_angle_reached)
    return new Teleop();

  return this;
}
 
/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head::state */
