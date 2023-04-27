/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_cyphal_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ros2_cyphal_bridge/types/Joint.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

std::string JointToStr(Joint const joint)
{
  switch(joint)
  {
    case Joint::Femur: return std::string("femur"); break;
    case Joint::Tibia: return std::string("tibia"); break;
    default: __builtin_unreachable();
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
