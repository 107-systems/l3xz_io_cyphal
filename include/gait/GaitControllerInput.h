/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_STATE_INPUT_H_
#define GAIT_CONTROLLER_STATE_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Const.h>
#include <common/sensor/interface/AnglePositionSensor.h>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class GaitControllerInput
{
public:
  GaitControllerInput(TeleopCommandData const teleop_cmd,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_front_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_front_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_middle_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_middle_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_back_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_back_right)
  : _teleop_cmd{teleop_cmd}
  , _angle_sensor_coxa_leg_front_left  {angle_sensor_coxa_leg_front_left}
  , _angle_sensor_coxa_leg_front_right {angle_sensor_coxa_leg_front_right}
  , _angle_sensor_coxa_leg_middle_left {angle_sensor_coxa_leg_middle_left}
  , _angle_sensor_coxa_leg_middle_right{angle_sensor_coxa_leg_middle_right}
  , _angle_sensor_coxa_leg_back_left   {angle_sensor_coxa_leg_back_left}
  , _angle_sensor_coxa_leg_back_right  {angle_sensor_coxa_leg_back_right}
  { }

  TeleopCommandData const _teleop_cmd;
  common::sensor::interface::SharedAnglePositionSensor _angle_sensor_coxa_leg_front_left,
                                                       _angle_sensor_coxa_leg_front_right,
                                                       _angle_sensor_coxa_leg_middle_left,
                                                       _angle_sensor_coxa_leg_middle_right,
                                                       _angle_sensor_coxa_leg_back_left,
                                                       _angle_sensor_coxa_leg_back_right;
};

#endif /* GAIT_CONTROLLER_STATE_INPUT_H_ */
