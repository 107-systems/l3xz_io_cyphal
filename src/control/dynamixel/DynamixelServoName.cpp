/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/control/dynamixel/DynamixelServoName.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

using namespace dynamixelplusplus;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

DynamixelServoName toServoName(Dynamixel::Id const id)
{
  static std::map<Dynamixel::Id, DynamixelServoName> const DYNAMIXEL_ID_TO_SERVO_NAME =
  {
    {1, DynamixelServoName::LeftFront_Coxa},
    {2, DynamixelServoName::LeftMiddle_Coxa},
    {3, DynamixelServoName::LeftBack_Coxa},
    {4, DynamixelServoName::RightBack_Coxa},
    {5, DynamixelServoName::RightMiddle_Coxa},
    {6, DynamixelServoName::RightFront_Coxa},
    {7, DynamixelServoName::Head_Pan},
    {8, DynamixelServoName::Head_Tilt},
  };

  return DYNAMIXEL_ID_TO_SERVO_NAME.at(id);
}

Dynamixel::Id toServoId(DynamixelServoName const name)
{
  static std::map<DynamixelServoName, Dynamixel::Id> const DYNAMIXEL_SERVO_NAME_TO_SERVO_ID =
  {
    {DynamixelServoName::LeftFront_Coxa,   1},
    {DynamixelServoName::LeftMiddle_Coxa,  2},
    {DynamixelServoName::LeftBack_Coxa,    3},
    {DynamixelServoName::RightBack_Coxa,   4},
    {DynamixelServoName::RightMiddle_Coxa, 5},
    {DynamixelServoName::RightFront_Coxa,  6},
    {DynamixelServoName::Head_Pan,         7},
    {DynamixelServoName::Head_Tilt,        8},
  };

  return DYNAMIXEL_SERVO_NAME_TO_SERVO_ID.at(name);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */
