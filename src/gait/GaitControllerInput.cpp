/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/GaitControllerInput.h>

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

struct valve_map_key_equal : public std::binary_function<GaitControllerInput::AngleSensorMapKey, GaitControllerInput::AngleSensorMapKey, bool>
{
  bool operator()(const GaitControllerInput::AngleSensorMapKey & v0, const GaitControllerInput::AngleSensorMapKey & v1) const
  {
    return (
            std::get<0>(v0) == std::get<0>(v1) &&
            std::get<1>(v0) == std::get<1>(v1)
           );
  }
};

static GaitControllerInput::AngleSensorMapKey make_key(Leg const leg, Joint const joint)
{
  return std::tuple(leg, joint);
}

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

GaitControllerInput::GaitControllerInput(TeleopCommandData const teleop_cmd,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_front_coxa,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_front_femur,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_front_tibia,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_middle_coxa,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_middle_femur,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_middle_tibia,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_back_coxa,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_back_femur,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_back_tibia,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_front_coxa,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_front_femur,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_front_tibia,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_middle_coxa,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_middle_femur,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_middle_tibia,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_back_coxa,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_back_femur,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_back_tibia)
: _teleop_cmd{teleop_cmd}
{
  _map[make_key(Leg::FrontLeft,   Joint::Coxa)]  = angle_sensor_left_front_coxa;
  _map[make_key(Leg::FrontLeft,   Joint::Femur)] = angle_sensor_left_front_femur;
  _map[make_key(Leg::FrontLeft,   Joint::Tibia)] = angle_sensor_left_front_tibia;

  _map[make_key(Leg::MiddleLeft,  Joint::Coxa)]  = angle_sensor_left_middle_coxa;
  _map[make_key(Leg::MiddleLeft,  Joint::Femur)] = angle_sensor_left_middle_femur;
  _map[make_key(Leg::MiddleLeft,  Joint::Tibia)] = angle_sensor_left_middle_tibia;

  _map[make_key(Leg::FrontLeft,   Joint::Coxa)]  = angle_sensor_left_front_coxa;
  _map[make_key(Leg::FrontLeft,   Joint::Femur)] = angle_sensor_left_front_femur;
  _map[make_key(Leg::FrontLeft,   Joint::Tibia)] = angle_sensor_left_front_tibia;

  _map[make_key(Leg::FrontRight,  Joint::Coxa)]  = angle_sensor_right_front_coxa;
  _map[make_key(Leg::FrontRight,  Joint::Femur)] = angle_sensor_right_front_femur;
  _map[make_key(Leg::FrontRight,  Joint::Tibia)] = angle_sensor_right_front_tibia;

  _map[make_key(Leg::MiddleRight, Joint::Coxa)]  = angle_sensor_right_middle_coxa;
  _map[make_key(Leg::MiddleRight, Joint::Femur)] = angle_sensor_right_middle_femur;
  _map[make_key(Leg::MiddleRight, Joint::Tibia)] = angle_sensor_right_middle_tibia;

  _map[make_key(Leg::FrontRight,  Joint::Coxa)]  = angle_sensor_right_front_coxa;
  _map[make_key(Leg::FrontRight,  Joint::Femur)] = angle_sensor_right_front_femur;
  _map[make_key(Leg::FrontRight,  Joint::Tibia)] = angle_sensor_right_front_tibia;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

common::sensor::interface::SharedAnglePositionSensor GaitControllerInput::operator()(Leg const leg, Joint const joint)
{
  if (!_map.count(make_key(leg, joint)))
  {
    std::stringstream msg;

    msg << "GaitControllerInput::operator(): error, trying to access non-existing position sensor ("
        << static_cast<int>(leg)
        << ", "
        << static_cast<int>(joint)
        << ")";

    throw std::runtime_error(msg.str());
  }

  return _map.at(make_key(leg, joint));
}

std::string GaitControllerInput::toStr()
{
  std::stringstream msg;

  msg << "\n"
      << "Left\n"
      << "  Front :"
      << "  Coxa: "   << _map[make_key(Leg::FrontLeft, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::FrontLeft, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::FrontLeft, Joint::Tibia)]->toStr()
      << "\n"
      << "  Middle:"
      << "  Coxa: "   << _map[make_key(Leg::MiddleLeft, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::MiddleLeft, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::MiddleLeft, Joint::Tibia)]->toStr()
      << "\n"
      << "  Back  :"
      << "  Coxa: "   << _map[make_key(Leg::BackLeft, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::BackLeft, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::BackLeft, Joint::Tibia)]->toStr()
      << "\n"
      << "Right\n"
      << "  Front :"
      << "  Coxa: "   << _map[make_key(Leg::FrontRight, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::FrontRight, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::FrontRight, Joint::Tibia)]->toStr()
      << "\n"
      << "  Middle:"
      << "  Coxa: "   << _map[make_key(Leg::MiddleRight, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::MiddleRight, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::MiddleRight, Joint::Tibia)]->toStr()
      << "\n"
      << "  Back  :"
      << "  Coxa: "   << _map[make_key(Leg::BackRight, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::BackRight, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::BackRight, Joint::Tibia)]->toStr();

  return msg.str();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */
