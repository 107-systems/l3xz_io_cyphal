/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef GLUE_DYNAMIXEL_SERVO_NAME_H_
#define GLUE_DYNAMIXEL_SERVO_NAME_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <dynamixel++/Dynamixel++.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class DynamixelServoName
{
  LeftFront_Coxa,
  LeftMiddle_Coxa,
  LeftBack_Coxa,
  RightBack_Coxa,
  RightMiddle_Coxa,
  RightFront_Coxa,
  Head_Pan,
  Head_Tilt,
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

DynamixelServoName toServoName(dynamixelplusplus::Dynamixel::Id const id);
dynamixelplusplus::Dynamixel::Id toServoId(DynamixelServoName const name);

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* GLUE_DYNAMIXEL_SERVO_NAME_H_ */
