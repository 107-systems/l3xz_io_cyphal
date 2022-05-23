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

#include <map>
#include <tuple>
#include <string>

#include <Const.h>
#include <common/sensor/interface/AnglePositionSensor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class GaitControllerInput
{
public:
  GaitControllerInput(TeleopCommandData const teleop_cmd,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_front_coxa,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_front_femur,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_front_tibia,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_middle_coxa,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_middle_femur,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_middle_tibia,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_back_coxa,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_back_femur,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_left_back_tibia,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_front_coxa,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_front_femur,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_front_tibia,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_middle_coxa,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_middle_femur,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_middle_tibia,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_back_coxa,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_back_femur,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_right_back_tibia);

  bool isValid() const;

  TeleopCommandData const _teleop_cmd;

  common::sensor::interface::SharedAnglePositionSensor operator()(Leg const leg, Joint const joint);

  std::string toStr();

  typedef std::tuple<Leg, Joint> AngleSensorMapKey;
  typedef common::sensor::interface::SharedAnglePositionSensor AngleSensorMapValue;
  typedef std::map<AngleSensorMapKey, AngleSensorMapValue> AngleSensorMap;

private:
  AngleSensorMap _map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */

#endif /* GAIT_CONTROLLER_STATE_INPUT_H_ */
