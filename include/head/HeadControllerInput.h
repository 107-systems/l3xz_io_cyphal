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

class ControllerInput
{
public:
  ControllerInput(TeleopCommandData const teleop_cmd,
                  common::sensor::interface::SharedAnglePositionSensor angle_sensor_sensor_head_pan,
                  common::sensor::interface::SharedAnglePositionSensor angle_sensor_sensor_head_tilt)
  : _teleop_cmd                     {teleop_cmd}
  , _angle_sensor_sensor_head_pan {angle_sensor_sensor_head_pan}
  , _angle_sensor_sensor_head_tilt{angle_sensor_sensor_head_tilt}
  { }

  bool isValid() const
  {
    if (!_angle_sensor_sensor_head_pan->get().has_value()) return false;
    if (!_angle_sensor_sensor_head_tilt->get().has_value()) return false;
    return true;
  }

  TeleopCommandData const _teleop_cmd;
  common::sensor::interface::SharedAnglePositionSensor _angle_sensor_sensor_head_pan,
                                                       _angle_sensor_sensor_head_tilt;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head */

#endif /* HEAD_CONTROLLER_INPUT_H_ */
