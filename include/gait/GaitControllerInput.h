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
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_front_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_front_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_middle_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_middle_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_back_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_coxa_leg_back_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_front_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_front_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_middle_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_middle_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_back_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_back_left,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_front_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_front_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_middle_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_middle_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_femur_leg_back_right,
                      common::sensor::interface::SharedAnglePositionSensor angle_sensor_tibia_leg_back_right);

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
