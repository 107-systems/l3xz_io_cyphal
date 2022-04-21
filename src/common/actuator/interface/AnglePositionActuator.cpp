/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/common/actuator/interface/AnglePositionActuator.h>

#include <sstream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

AnglePositionActuator::AnglePositionActuator(std::string const & name)
: _name{name}
, _val{std::nullopt}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void AnglePositionActuator::set(float const val)
{
  _val = val;
}

std::string AnglePositionActuator::toStr() const
{
  std::stringstream ss;
  ss << "[A] "
     << _name << ": ";
  
  if (_val)
    ss << _val.value();
  else
    ss << "Inv.";

  return ss.str();
}

/**************************************************************************************
 * PROTECTED MEMBER FUNCTIONS
 **************************************************************************************/

std::optional<float> AnglePositionActuator::get() const
{
  return _val;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */
