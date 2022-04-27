/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef ROBOT_STATE_OUTPUT_H_
#define ROBOT_STATE_OUTPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/AnglePositionActuator.h>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class RobotStateOutput
{
public:
  RobotStateOutput(common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_front_left,
                   common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_front_right,
                   common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_middle_left,
                   common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_middle_right,
                   common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_back_left,
                   common::actuator::interface::SharedAnglePositionActuator angle_actuator_coxa_leg_back_right)
  : _angle_actuator_coxa_leg_front_left  {angle_actuator_coxa_leg_front_left}
  , _angle_actuator_coxa_leg_front_right {angle_actuator_coxa_leg_front_right}
  , _angle_actuator_coxa_leg_middle_left {angle_actuator_coxa_leg_middle_left}
  , _angle_actuator_coxa_leg_middle_right{angle_actuator_coxa_leg_middle_right}
  , _angle_actuator_coxa_leg_back_left   {angle_actuator_coxa_leg_back_left}
  , _angle_actuator_coxa_leg_back_right  {angle_actuator_coxa_leg_back_right}
  { }

  common::actuator::interface::SharedAnglePositionActuator _angle_actuator_coxa_leg_front_left,
                                                           _angle_actuator_coxa_leg_front_right,
                                                           _angle_actuator_coxa_leg_middle_left,
                                                           _angle_actuator_coxa_leg_middle_right,
                                                           _angle_actuator_coxa_leg_back_left,
                                                           _angle_actuator_coxa_leg_back_right;
};

#endif /* ROBOT_STATE_OUTPUT_H_ */
