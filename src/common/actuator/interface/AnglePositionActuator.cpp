/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/AnglePositionActuator.h>

#include <sstream>
#include <iomanip>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

std::string AnglePositionActuator::toStr() const
{
  std::stringstream ss;

  if (get().has_value())
    ss << std::fixed
       << std::setprecision(2) 
       << std::setfill(' ')
       << std::setw(6)
       << get().value();
  else
    ss << "  Inv.";

  return ss.str();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */
