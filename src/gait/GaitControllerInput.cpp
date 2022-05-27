/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/GaitControllerInput.h>

#include <Const.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ControllerInput::ControllerInput(TeleopCommandData const teleop_cmd,
                                 std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> const & angle_position_sensor_map,
                                 std::map<Leg, common::sensor::interface::SharedBumperSensor> const & bumper_sensor_map)
: _teleop_cmd{teleop_cmd}
{
  for (auto [leg, joint] : LEG_JOINT_LIST)
    _angle_position_map[make_key(leg, joint)] = angle_position_sensor_map.at(make_key(leg, joint))->get().value();

  for (auto leg : LEG_LIST)
    _bumper_map[leg] = bumper_sensor_map.at(leg)->isPressed().value();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

float ControllerInput::get_angle_deg(Leg const leg, Joint const joint) const
{
  return _angle_position_map.at(make_key(leg, joint));
}

bool ControllerInput::isBumperPressed(Leg const leg) const
{
  return _bumper_map.at(leg);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */
