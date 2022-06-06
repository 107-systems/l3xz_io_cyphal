/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <sstream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

template <typename T>
std::string Base<T>::toStr() const
{
  std::stringstream ss;

  ss << "[A] "
      << _name << ": ";

  if (get().has_value())
    ss << get().value();
  else
    ss << "Inv.";

  return ss.str();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */
