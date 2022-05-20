/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/GaitControllerOutput.h>

#include <sstream>
#include <stdexcept>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

struct valve_map_key_equal : public std::binary_function<GaitControllerOutput::AngleActuatorMapKey, GaitControllerOutput::AngleActuatorMapKey, bool>
{
  bool operator()(const GaitControllerOutput::AngleActuatorMapKey & v0, const GaitControllerOutput::AngleActuatorMapKey & v1) const
  {
    return (
            std::get<0>(v0) == std::get<0>(v1) &&
            std::get<1>(v0) == std::get<1>(v1)
           );
  }
};

static GaitControllerOutput::AngleActuatorMapKey make_key(Leg const leg, Joint const joint)
{
  return std::tuple(leg, joint);
}

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

GaitControllerOutput::GaitControllerOutput(common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_front_coxa,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_front_femur,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_front_tibia,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_middle_coxa,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_middle_femur,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_middle_tibia,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_back_coxa,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_back_femur,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_back_tibia,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_front_coxa,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_front_femur,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_front_tibia,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_middle_coxa,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_middle_femur,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_middle_tibia,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_back_coxa,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_back_femur,
                                           common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_back_tibia)
{
  _map[make_key(Leg::FrontLeft,   Joint::Coxa)]  = angle_actuator_left_front_coxa;
  _map[make_key(Leg::FrontLeft,   Joint::Femur)] = angle_actuator_left_front_femur;
  _map[make_key(Leg::FrontLeft,   Joint::Tibia)] = angle_actuator_left_front_tibia;

  _map[make_key(Leg::MiddleLeft,  Joint::Coxa)]  = angle_actuator_left_middle_coxa;
  _map[make_key(Leg::MiddleLeft,  Joint::Femur)] = angle_actuator_left_middle_femur;
  _map[make_key(Leg::MiddleLeft,  Joint::Tibia)] = angle_actuator_left_middle_tibia;

  _map[make_key(Leg::FrontLeft,   Joint::Coxa)]  = angle_actuator_left_front_coxa;
  _map[make_key(Leg::FrontLeft,   Joint::Femur)] = angle_actuator_left_front_femur;
  _map[make_key(Leg::FrontLeft,   Joint::Tibia)] = angle_actuator_left_front_tibia;

  _map[make_key(Leg::FrontRight,  Joint::Coxa)]  = angle_actuator_right_front_coxa;
  _map[make_key(Leg::FrontRight,  Joint::Femur)] = angle_actuator_right_front_femur;
  _map[make_key(Leg::FrontRight,  Joint::Tibia)] = angle_actuator_right_front_tibia;

  _map[make_key(Leg::MiddleRight, Joint::Coxa)]  = angle_actuator_right_middle_coxa;
  _map[make_key(Leg::MiddleRight, Joint::Femur)] = angle_actuator_right_middle_femur;
  _map[make_key(Leg::MiddleRight, Joint::Tibia)] = angle_actuator_right_middle_tibia;

  _map[make_key(Leg::FrontRight,  Joint::Coxa)]  = angle_actuator_right_front_coxa;
  _map[make_key(Leg::FrontRight,  Joint::Femur)] = angle_actuator_right_front_femur;
  _map[make_key(Leg::FrontRight,  Joint::Tibia)] = angle_actuator_right_front_tibia;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

common::actuator::interface::SharedAnglePositionActuator GaitControllerOutput::operator()(Leg const leg, Joint const joint)
{
  if (!_map.count(make_key(leg, joint)))
  {
    std::stringstream msg;

    msg << "GaitControllerOutput::operator(): error, trying to access non-existing position actuator ("
        << static_cast<int>(leg)
        << ", "
        << static_cast<int>(joint)
        << ")";

    throw std::runtime_error(msg.str());
  }

  return _map.at(make_key(leg, joint));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */
