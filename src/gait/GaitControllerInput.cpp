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
 * CTOR/DTOR
 **************************************************************************************/

GaitControllerInput::GaitControllerInput(TeleopCommandData const teleop_cmd,
                                         std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> const & angle_position_sensor_map)
: _teleop_cmd{teleop_cmd}
{
  for (auto [leg, joint] : LEG_JOINT_LIST)
    _angle_position_map[make_key(leg, joint)] = angle_position_sensor_map.at(make_key(leg, joint))->get().value();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

float GaitControllerInput::operator()(Leg const leg, Joint const joint) const
{
  if (!_angle_position_map.count(make_key(leg, joint)))
  {
    std::stringstream msg;

    msg << "GaitControllerInput::operator(): error, trying to access non-existing position sensor ("
        << static_cast<int>(leg)
        << ", "
        << static_cast<int>(joint)
        << ")";

    throw std::runtime_error(msg.str());
  }

  return _angle_position_map.at(make_key(leg, joint));
}
/*
std::string GaitControllerInput::toStr()
{
  std::stringstream msg;

  msg << "\n"
      << "Left\n"
      << "  Front :"
      << "  Coxa: "   << _map[make_key(Leg::LeftFront, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::LeftFront, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::LeftFront, Joint::Tibia)]->toStr()
      << "\n"
      << "  Middle:"
      << "  Coxa: "   << _map[make_key(Leg::LeftMiddle, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::LeftMiddle, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::LeftMiddle, Joint::Tibia)]->toStr()
      << "\n"
      << "  Back  :"
      << "  Coxa: "   << _map[make_key(Leg::LeftBack, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::LeftBack, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::LeftBack, Joint::Tibia)]->toStr()
      << "\n"
      << "Right\n"
      << "  Front :"
      << "  Coxa: "   << _map[make_key(Leg::RightFront, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::RightFront, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::RightFront, Joint::Tibia)]->toStr()
      << "\n"
      << "  Middle:"
      << "  Coxa: "   << _map[make_key(Leg::RightMiddle, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::RightMiddle, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::RightMiddle, Joint::Tibia)]->toStr()
      << "\n"
      << "  Back  :"
      << "  Coxa: "   << _map[make_key(Leg::RightBack, Joint::Coxa)]->toStr()
      << "  Femur: "  << _map[make_key(Leg::RightBack, Joint::Femur)]->toStr()
      << "  Tibia: "  << _map[make_key(Leg::RightBack, Joint::Tibia)]->toStr();

  return msg.str();
}
*/
/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */
