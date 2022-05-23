/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <common/kinematic/IK_Output.h>

#include <sstream>
#include <iomanip>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::kinematic
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

IK_Output::IK_Output(double const coxa_angle_rad, double const femur_angle_rad, double const tibia_angle_rad)
: _coxa_angle_deg {coxa_angle_rad  * 180.0 / M_PI}
, _femur_angle_deg{femur_angle_rad * 180.0 / M_PI}
, _tibia_angle_deg{tibia_angle_rad * 180.0 / M_PI}
{ }

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::string IK_Output::toStr() const
{
  std::stringstream msg;

  msg << "("
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _coxa_angle_deg
      << ", "
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _femur_angle_deg
      << ", "
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _tibia_angle_deg;

  return msg.str();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */
