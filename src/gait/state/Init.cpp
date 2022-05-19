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

#include <gait/state/Standing.h>

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

StateBase * Init::update(GaitControllerInput & input, GaitControllerOutput & output)
{
  static std::list<GaitControllerInput::AngleSensorMapKey> const COXA_ANGLE_SENSOR_KEY_LIST =
  {
    std::tuple(Leg::FrontLeft,   Joint::Coxa),
    std::tuple(Leg::FrontRight,  Joint::Coxa),
    std::tuple(Leg::MiddleLeft,  Joint::Coxa),
    std::tuple(Leg::MiddleRight, Joint::Coxa),
    std::tuple(Leg::MiddleLeft,  Joint::Coxa),
    std::tuple(Leg::MiddleRight, Joint::Coxa),
  };


  /* Set the desired target angle. */
  output(Leg::FrontLeft,   Joint::Coxa)->set(INITIAL_COXA_ANGLE);
  output(Leg::FrontRight,  Joint::Coxa)->set(INITIAL_COXA_ANGLE);
  output(Leg::MiddleLeft,  Joint::Coxa)->set(INITIAL_COXA_ANGLE);
  output(Leg::MiddleRight, Joint::Coxa)->set(INITIAL_COXA_ANGLE);
  output(Leg::BackLeft,    Joint::Coxa)->set(INITIAL_COXA_ANGLE);
  output(Leg::BackRight,   Joint::Coxa)->set(INITIAL_COXA_ANGLE);


  /* Check if we have valid angles. */
  for (auto [leg, joint] : COXA_ANGLE_SENSOR_KEY_LIST)
  {
    if (!input(leg, joint)->get().has_value()) {
      ROS_ERROR("gait::state::Init::update: no valid input data for %s", input(leg, joint)->name().c_str());
      return this;
    }
  }


  /* Check if we have reached the desired target angle. */
  for (auto [leg, joint] : COXA_ANGLE_SENSOR_KEY_LIST)
  {
    float const coxa_angle_actual = input(leg, joint)->get().value();
    float const coxa_angle_diff = fabs(INITIAL_COXA_ANGLE - coxa_angle_actual);
    bool  const coxa_is_initial_angle_reached = coxa_angle_diff < 1.0f;

    if (!coxa_is_initial_angle_reached) {
      ROS_INFO("gait::state::Init::update: target angle not reached for %s", input(leg, joint)->name().c_str());
      return this;
    }
  }

  /* All good, let's transition to the next state. */
  return new Standing();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
