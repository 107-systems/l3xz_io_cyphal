/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_INPUT_H_
#define HEAD_CONTROLLER_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Const.h>
#include <common/sensor/interface/AnglePositionSensor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class HeadControllerInput
{
public:
  HeadControllerInput(TeleopCommandData const teleop_cmd,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_sensor_head_pan,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_sensor_head_tilt)
  : _teleop_cmd                     {teleop_cmd}
  , _angle_sensor_sensor_head_pan {angle_sensor_sensor_head_pan}
  , _angle_sensor_sensor_head_tilt{angle_sensor_sensor_head_tilt}
  { }

  TeleopCommandData const _teleop_cmd;
  common::sensor::interface::SharedAnglePositionSensor _angle_sensor_sensor_head_pan,
                                                       _angle_sensor_sensor_head_tilt;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head */

#endif /* HEAD_CONTROLLER_INPUT_H_ */
