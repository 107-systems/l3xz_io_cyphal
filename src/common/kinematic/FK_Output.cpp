/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <common/kinematic/FK_Output.h>

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

FK_Output::FK_Output(double const tibia_tip_x, double const tibia_tip_y, double const tibia_tip_z)
: _tibia_tip_x{tibia_tip_x}
, _tibia_tip_y{tibia_tip_y}
, _tibia_tip_z{tibia_tip_z}
{ }

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::string FK_Output::toStr() const
{
  std::stringstream msg;

  msg << "("
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _tibia_tip_x
      << ", "
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _tibia_tip_y
      << ", "
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _tibia_tip_z;

  return msg.str();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */
