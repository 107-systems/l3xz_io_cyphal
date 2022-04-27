/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef ROBOT_STATE_INPUT_H_
#define ROBOT_STATE_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/sensor/interface/AnglePositionSensor.h>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class RobotStateInput
{
public:
  RobotStateInput(common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_front_left,
                  common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_front_right,
                  common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_middle_left,
                  common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_middle_right,
                  common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_back_left,
                  common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_back_right)
  : _angle_sensor_coxa_leg_front_left  {angle_sensor_coxa_leg_front_left}
  , _angle_sensor_coxa_leg_front_right {angle_sensor_coxa_leg_front_right}
  , _angle_sensor_coxa_leg_middle_left {angle_sensor_coxa_leg_middle_left}
  , _angle_sensor_coxa_leg_middle_right{angle_sensor_coxa_leg_middle_right}
  , _angle_sensor_coxa_leg_back_left   {angle_sensor_coxa_leg_back_left}
  , _angle_sensor_coxa_leg_back_right  {angle_sensor_coxa_leg_back_right}
  { }

  common::sensor::interface::SharedAnglePositionSensor _angle_sensor_coxa_leg_front_left,
                                                       _angle_sensor_coxa_leg_front_right,
                                                       _angle_sensor_coxa_leg_middle_left,
                                                       _angle_sensor_coxa_leg_middle_right,
                                                       _angle_sensor_coxa_leg_back_left,
                                                       _angle_sensor_coxa_leg_back_right;
};

#endif /* ROBOT_STATE_INPUT_H_ */
