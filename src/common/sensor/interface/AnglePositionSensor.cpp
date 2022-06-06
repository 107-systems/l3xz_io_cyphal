/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/sensor/interface/AnglePositionSensor.h>

#include <sstream>
#include <iomanip>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::sensor::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

std::string AnglePositionSensor::toStr() const
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

} /* common::sensor::interface */
