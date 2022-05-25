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

struct valve_map_key_equal : public std::binary_function<GaitControllerOutput::TargetAngleMapKey, GaitControllerOutput::TargetAngleMapKey, bool>
{
  bool operator()(const GaitControllerOutput::TargetAngleMapKey & v0, const GaitControllerOutput::TargetAngleMapKey & v1) const
  {
    return (
            std::get<0>(v0) == std::get<0>(v1) &&
            std::get<1>(v0) == std::get<1>(v1)
           );
  }
};

static GaitControllerOutput::TargetAngleMapKey make_key(Leg const leg, Joint const joint)
{
  return std::tuple(leg, joint);
}

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

GaitControllerOutput::GaitControllerOutput(float const left_front_coxa_angle_target,
                                           float const left_front_femur_angle_target,
                                           float const left_front_tibia_angle_target,
                                           float const left_middle_coxa_angle_target,
                                           float const left_middle_femur_angle_target,
                                           float const left_middle_tibia_angle_target,
                                           float const left_back_coxa_angle_target,
                                           float const left_back_femur_angle_target,
                                           float const left_back_tibia_angle_target,
                                           float const right_front_coxa_angle_target,
                                           float const right_front_femur_angle_target,
                                           float const right_front_tibia_angle_target,
                                           float const right_middle_coxa_angle_target,
                                           float const right_middle_femur_angle_target,
                                           float const right_middle_tibia_angle_target,
                                           float const right_back_coxa_angle_target,
                                           float const right_back_femur_angle_target,
                                           float const right_back_tibia_angle_target)
{
  _map[make_key(Leg::LeftFront,   Joint::Coxa)]  = left_front_coxa_angle_target;
  _map[make_key(Leg::LeftFront,   Joint::Femur)] = left_front_femur_angle_target;
  _map[make_key(Leg::LeftFront,   Joint::Tibia)] = left_front_tibia_angle_target;

  _map[make_key(Leg::LeftMiddle,  Joint::Coxa)]  = left_middle_coxa_angle_target;
  _map[make_key(Leg::LeftMiddle,  Joint::Femur)] = left_middle_femur_angle_target;
  _map[make_key(Leg::LeftMiddle,  Joint::Tibia)] = left_middle_tibia_angle_target;

  _map[make_key(Leg::LeftBack,    Joint::Coxa)]  = left_back_coxa_angle_target;
  _map[make_key(Leg::LeftBack,    Joint::Femur)] = left_back_femur_angle_target;
  _map[make_key(Leg::LeftBack,    Joint::Tibia)] = left_back_tibia_angle_target;

  _map[make_key(Leg::RightFront,  Joint::Coxa)]  = right_front_coxa_angle_target;
  _map[make_key(Leg::RightFront,  Joint::Femur)] = right_front_femur_angle_target;
  _map[make_key(Leg::RightFront,  Joint::Tibia)] = right_front_tibia_angle_target;

  _map[make_key(Leg::RightMiddle, Joint::Coxa)]  = right_middle_coxa_angle_target;
  _map[make_key(Leg::RightMiddle, Joint::Femur)] = right_middle_femur_angle_target;
  _map[make_key(Leg::RightMiddle, Joint::Tibia)] = right_middle_tibia_angle_target;

  _map[make_key(Leg::RightBack,   Joint::Coxa)]  = right_back_coxa_angle_target;
  _map[make_key(Leg::RightBack,   Joint::Femur)] = right_back_femur_angle_target;
  _map[make_key(Leg::RightBack,   Joint::Tibia)] = right_back_tibia_angle_target;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

float & GaitControllerOutput::at(Leg const leg, Joint const joint)
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

std::string GaitControllerOutput::toStr()
{
  std::stringstream msg;

  msg << "\n"
      << "Left\n"
      << "  Front :"
      << "  Coxa: "   << _map[make_key(Leg::LeftFront, Joint::Coxa)]
      << "  Femur: "  << _map[make_key(Leg::LeftFront, Joint::Femur)]
      << "  Tibia: "  << _map[make_key(Leg::LeftFront, Joint::Tibia)]
      << "\n"
      << "  Middle:"
      << "  Coxa: "   << _map[make_key(Leg::LeftMiddle, Joint::Coxa)]
      << "  Femur: "  << _map[make_key(Leg::LeftMiddle, Joint::Femur)]
      << "  Tibia: "  << _map[make_key(Leg::LeftMiddle, Joint::Tibia)]
      << "\n"
      << "  Back  :"
      << "  Coxa: "   << _map[make_key(Leg::LeftBack, Joint::Coxa)]
      << "  Femur: "  << _map[make_key(Leg::LeftBack, Joint::Femur)]
      << "  Tibia: "  << _map[make_key(Leg::LeftBack, Joint::Tibia)]
      << "\n"
      << "Right\n"
      << "  Front :"
      << "  Coxa: "   << _map[make_key(Leg::RightFront, Joint::Coxa)]
      << "  Femur: "  << _map[make_key(Leg::RightFront, Joint::Femur)]
      << "  Tibia: "  << _map[make_key(Leg::RightFront, Joint::Tibia)]
      << "\n"
      << "  Middle:"
      << "  Coxa: "   << _map[make_key(Leg::RightMiddle, Joint::Coxa)]
      << "  Femur: "  << _map[make_key(Leg::RightMiddle, Joint::Femur)]
      << "  Tibia: "  << _map[make_key(Leg::RightMiddle, Joint::Tibia)]
      << "\n"
      << "  Back  :"
      << "  Coxa: "   << _map[make_key(Leg::RightBack, Joint::Coxa)]
      << "  Femur: "  << _map[make_key(Leg::RightBack, Joint::Femur)]
      << "  Tibia: "  << _map[make_key(Leg::RightBack, Joint::Tibia)];

  return msg.str();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */
