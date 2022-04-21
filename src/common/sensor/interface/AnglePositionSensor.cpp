/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/common/sensor/interface/AnglePositionSensor.h>

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
: _name{name}
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
  ss << _name << ": ";
  
  if (_val)
    ss << _val.value();
  else
    ss << "Inv.";

  return ss.str();
}

/**************************************************************************************
 * PROTECTED MEMBER FUNCTIONS
 **************************************************************************************/

void AnglePositionSensor::update(float const val)
{
  _val = val;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::sensor::interface */
