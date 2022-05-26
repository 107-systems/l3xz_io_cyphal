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

class ControllerOutput
{
public:
  ControllerOutput(float const left_front_coxa_angle_target,
                   float const left_front_femur_angle_target,
                   float const left_front_tibia_angle_target,
                   float const left_middle_coxa_angle_target,
                   float const left_middle_femur_angle_target,
                   float const left_middle_tibia_angle_target,
                   float const left_back_coxa_angle_target,
                   float const left_back_femur_angle_target,
                   float const left_back_tibia_angle_target,
                   float const right_front_coxa_angle_target,
                   float const right_front_femur_angle_target,
                   float const right_front_tibia_angle_target,
                   float const right_middle_coxa_angle_target,
                   float const right_middle_femur_angle_target,
                   float const right_middle_tibia_angle_target,
                   float const right_back_coxa_angle_target,
                   float const right_back_femur_angle_target,
                   float const right_back_tibia_angle_target);

  float & at(Leg const leg, Joint const joint);

  std::string toStr() const;

private:
  std::map<LegJointKey, float> _angle_position_map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */

#endif /* GAIT_CONTROLLER_STATE_OUTPUT_H_ */
