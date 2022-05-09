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

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::sensor::interface
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

AnglePositionSensor::AnglePositionSensor(std::string const & name)
: Base(name)
, _val{std::nullopt}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::optional<float> AnglePositionSensor::get() const
{
  return _val;
}

std::string AnglePositionSensor::toStr() const
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

void AnglePositionSensor::set(float const val)
{
  _val = val;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::sensor::interface */
