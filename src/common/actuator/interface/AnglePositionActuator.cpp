/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/AnglePositionActuator.h>

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
: Base(name)
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
  ss << Base::toStr();
  
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
