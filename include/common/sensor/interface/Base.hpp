/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef COMMON_SENSOR_INTERFACE_BASE_HPP_
#define COMMON_SENSOR_INTERFACE_BASE_HPP_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>
#include <optional>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::sensor::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

template <typename T>
class Base
{
public:
           Base(std::string const & name) : _name{name} { }
  virtual ~Base() { }

  virtual std::optional<T> get() const = 0;
  std::string toStr() const;

protected:
  inline std::string name() const { return _name; }

private:
  std::string const _name;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::sensor::interface */

/**************************************************************************************
 * TEMPLATE IMPLEMENTATION
 **************************************************************************************/

#include "Base.ipp"

#endif /* COMMON_SENSOR_INTERFACE_BASE_HPP_ */
