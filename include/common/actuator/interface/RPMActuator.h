/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef COMMON_ACTUATOR_INTERFACE_RPM_ACTUATOR_H_
#define COMMON_ACTUATOR_INTERFACE_RPM_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Base.hpp"

#include <memory>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class RPMActuator : public Base<uint32_t>
{
public:
           RPMActuator(std::string const & name) : Base{std::string("[RPM Actuator] \"") + name + std::string("\"")} { }
  virtual ~RPMActuator() { }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

typedef std::shared_ptr<RPMActuator> SharedRPMActuator;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */

#endif /* COMMON_ACTUATOR_INTERFACE_RPM_ACTUATOR_H_ */
