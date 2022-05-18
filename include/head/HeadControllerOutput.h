/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_OUTPUT_H_
#define HEAD_CONTROLLER_OUTPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/AnglePositionActuator.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class HeadControllerOutput
{
public:
  HeadControllerOutput(common::actuator::interface::SharedAnglePositionActuator angle_actuator_sensor_head_pan,
                       common::actuator::interface::SharedAnglePositionActuator angle_actuator_sensor_head_tilt)
  : _angle_actuator_sensor_head_pan {angle_actuator_sensor_head_pan}
  , _angle_actuator_sensor_head_tilt{angle_actuator_sensor_head_tilt}
  { }

  common::actuator::interface::SharedAnglePositionActuator _angle_actuator_sensor_head_pan,
                                                           _angle_actuator_sensor_head_tilt;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head */

#endif /* HEAD_CONTROLLER_OUTPUT_H_ */
