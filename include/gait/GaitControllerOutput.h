/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_STATE_OUTPUT_H_
#define GAIT_CONTROLLER_STATE_OUTPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>
#include <tuple>

#include <Const.h>
#include <common/actuator/interface/AnglePositionActuator.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class GaitControllerOutput
{
public:
  GaitControllerOutput(common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_front_left,
                       common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_front_right,
                       common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_middle_left,
                       common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_middle_right,
                       common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_back_left,
                       common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_back_right);

  common::actuator::interface::SharedAnglePositionActuator operator()(Leg const leg, Joint const joint);

  typedef std::tuple<Leg, Joint> AngleActuatorMapKey;
  typedef common::actuator::interface::SharedAnglePositionActuator AngleActuatorMapValue;
  typedef std::map<AngleActuatorMapKey, AngleActuatorMapValue> AngleActuatorMap;

private:
  AngleActuatorMap _map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */

#endif /* GAIT_CONTROLLER_STATE_OUTPUT_H_ */
