/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <head/HeadControllerOutput.h>

#include <cmath>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head
{

/**************************************************************************************
 * FREE FUNCTION DEFINITION
 **************************************************************************************/

bool operator == (ControllerOutput const & lhs, ControllerOutput const & rhs)
{
  static float const EPSILON = 1e-3;
  
  bool const is_pan_equal  = fabs(lhs[ControllerOutput::Angle::Pan]  - rhs[ControllerOutput::Angle::Pan])  < EPSILON;
  bool const is_tilt_equal = fabs(lhs[ControllerOutput::Angle::Tilt] - rhs[ControllerOutput::Angle::Tilt]) < EPSILON;

  return (is_pan_equal && is_tilt_equal);
}

bool operator != (ControllerOutput const & lhs, ControllerOutput const & rhs)
{
  return !(lhs == rhs);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head */
