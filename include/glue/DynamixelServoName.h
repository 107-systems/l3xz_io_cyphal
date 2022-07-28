/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_DYNAMIXEL_SERVO_NAME_H_
#define GLUE_DYNAMIXEL_SERVO_NAME_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <driver/dynamixel/Dynamixel.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
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

DynamixelServoName toServoName(dynamixel::Dynamixel::Id const id);
dynamixel::Dynamixel::Id toServoId(DynamixelServoName const name);

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_DYNAMIXEL_SERVO_NAME_H_ */
