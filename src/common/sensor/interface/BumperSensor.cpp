/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/sensor/interface/BumperSensor.h>

#include <sstream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::sensor::interface
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::string BumperSensor::toStr() const
{
  std::stringstream ss;

  if (!isPressed().has_value()) {
    ss << "      Inv.";
    return ss.str();
  }

  bool const is_pressed = isPressed().value();
  if (is_pressed)
    ss << "  Pressed.";
  else
    ss << " Released.";

  return ss.str();
}

std::optional<bool> BumperSensor::isPressed() const
{
  if (get().has_value())
    return get().value();
  else
    return std::nullopt;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::sensor::interface */
