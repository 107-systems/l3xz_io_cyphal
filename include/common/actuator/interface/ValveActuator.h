/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef COMMON_ACTUATOR_INTERFACE_VALVE_ACTUATOR_H_
#define COMMON_ACTUATOR_INTERFACE_VALVE_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>
#include <memory>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ValveActuator
{
public:
           ValveActuator(std::string const & name) : _name{name} { }
  virtual ~ValveActuator() { }

  virtual void set(float const val) = 0;
  virtual std::string toStr() const = 0;

protected:
  inline std::string name() const { return _name; }

private:
  std::string const _name;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

typedef std::shared_ptr<ValveActuator> SharedValveActuator;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */

#endif /* COMMON_ACTUATOR_INTERFACE_VALVE_ACTUATOR_H_ */
