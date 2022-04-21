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
, _val{0.0f}
, _is_valid{false}
{

}

AnglePositionSensor::~AnglePositionSensor()
{
  _is_valid = false;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::optional<float> AnglePositionSensor::get() const
{
  if (_is_valid)
    return _val;

  return std::nullopt;
}

void AnglePositionSensor::update(float const val)
{
  _val = val;
  _is_valid = true;
}

std::string AnglePositionSensor::toStr() const
{
  std::stringstream ss;
  ss << _name << ": ";
  
  if (_is_valid)
    ss << _val;
  else
    ss << "Inv.";

  return ss.str();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::sensor::interface */
