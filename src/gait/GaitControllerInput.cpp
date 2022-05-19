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
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_front_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_front_right,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_middle_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_middle_right,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_back_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_back_right,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_front_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_front_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_middle_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_middle_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_back_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_back_left,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_front_right,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_front_right,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_middle_right,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_middle_right,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_back_right,
                                         common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_back_right)
: _teleop_cmd{teleop_cmd}
{
  _map[make_key(Leg::FrontLeft,   Joint::Coxa)] = angle_sensor_coxa_leg_front_left;
  _map[make_key(Leg::FrontRight,  Joint::Coxa)] = angle_sensor_coxa_leg_front_right;
  _map[make_key(Leg::MiddleLeft,  Joint::Coxa)] = angle_sensor_coxa_leg_middle_left;
  _map[make_key(Leg::MiddleRight, Joint::Coxa)] = angle_sensor_coxa_leg_middle_right;
  _map[make_key(Leg::BackLeft,    Joint::Coxa)] = angle_sensor_coxa_leg_back_left;
  _map[make_key(Leg::BackRight,   Joint::Coxa)] = angle_sensor_coxa_leg_back_right;

  _map[make_key(Leg::FrontLeft,   Joint::Femur)] = angle_sensor_femur_leg_front_left;
  _map[make_key(Leg::FrontRight,  Joint::Femur)] = angle_sensor_femur_leg_front_right;
  _map[make_key(Leg::MiddleLeft,  Joint::Femur)] = angle_sensor_femur_leg_middle_left;
  _map[make_key(Leg::MiddleRight, Joint::Femur)] = angle_sensor_femur_leg_middle_right;
  _map[make_key(Leg::BackLeft,    Joint::Femur)] = angle_sensor_femur_leg_back_left;
  _map[make_key(Leg::BackRight,   Joint::Femur)] = angle_sensor_femur_leg_back_right;

  _map[make_key(Leg::FrontLeft,   Joint::Tibia)] = angle_sensor_tibia_leg_front_left;
  _map[make_key(Leg::FrontRight,  Joint::Tibia)] = angle_sensor_tibia_leg_front_right;
  _map[make_key(Leg::MiddleLeft,  Joint::Tibia)] = angle_sensor_tibia_leg_middle_left;
  _map[make_key(Leg::MiddleRight, Joint::Tibia)] = angle_sensor_tibia_leg_middle_right;
  _map[make_key(Leg::BackLeft,    Joint::Tibia)] = angle_sensor_tibia_leg_back_left;
  _map[make_key(Leg::BackRight,   Joint::Tibia)] = angle_sensor_tibia_leg_back_right;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

common::sensor::interface::SharedAnglePositionSensor GaitControllerInput::operator()(Leg const leg, Joint const joint)
{
  if (!_map.count(make_key(leg, joint)))
  {
    std::stringstream msg;

    msg << "GaitControllerInput::operator(): error, trying to access non-existing position actuator ("
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
