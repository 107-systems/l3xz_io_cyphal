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

ControllerInput::ControllerInput(TeleopCommandData const teleop_cmd,
                                 std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> const & angle_position_sensor_map)
: _teleop_cmd{teleop_cmd}
{
  for (auto [leg, joint] : LEG_JOINT_LIST)
    _angle_position_map[make_key(leg, joint)] = angle_position_sensor_map.at(make_key(leg, joint))->get().value();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

float ControllerInput::angle_deg(Leg const leg, Joint const joint) const
{
  if (!_angle_position_map.count(make_key(leg, joint)))
  {
    std::stringstream msg;

    msg << "ControllerInput::operator(): error, trying to access non-existing position sensor ("
        << static_cast<int>(leg)
        << ", "
        << static_cast<int>(joint)
        << ")";

    throw std::runtime_error(msg.str());
  }

  return _angle_position_map.at(make_key(leg, joint));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */
