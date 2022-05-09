/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef COMMON_ACTUATOR_INTERFACE_BASE_H_
#define COMMON_ACTUATOR_INTERFACE_BASE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>
#include <sstream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Base
{
public:
           Base(std::string const & name) : _name{name} { }
  virtual ~Base() { }

  virtual std::string toStr() const
  {
    std::stringstream ss;
    ss << "[A] "
        << name() << ": ";
    return ss.str();
  }

protected:
  inline std::string name() const { return _name; }

private:
  std::string const _name;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */

#endif /* COMMON_ACTUATOR_INTERFACE_BASE_H_ */
