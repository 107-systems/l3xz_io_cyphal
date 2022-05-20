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
#include <string>

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
  GaitControllerOutput(common::actuator::interface::SharedAnglePositionActuator angle_actuator_left_front_coxa,
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
                       common::actuator::interface::SharedAnglePositionActuator angle_actuator_right_back_tibia);
                       

  common::actuator::interface::SharedAnglePositionActuator operator()(Leg const leg, Joint const joint);

  std::string toStr();

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
