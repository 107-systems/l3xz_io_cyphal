/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_STATE_INPUT_H_
#define GAIT_CONTROLLER_STATE_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>

#include <Types.h>

#include <common/sensor/interface/BumperSensor.h>
#include <common/sensor/interface/AnglePositionSensor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ControllerInput
{
public:
  ControllerInput(TeleopCommandData const teleop_cmd,
                  std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> const & angle_position_sensor_map,
                  std::map<Leg, common::sensor::interface::SharedBumperSensor> const & bumper_sensor_map);

  TeleopCommandData teleop_cmd() const { return _teleop_cmd; }
  float             get_angle_deg(Leg const leg, Joint const joint) const;
  bool              isBumperPressed(Leg const leg) const;

private:
  TeleopCommandData const _teleop_cmd;
  std::map<LegJointKey, float> _angle_position_map;
  std::map<Leg, bool> _bumper_map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */

#endif /* GAIT_CONTROLLER_STATE_INPUT_H_ */
