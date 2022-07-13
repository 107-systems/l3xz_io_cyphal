/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef COMMON_ACTUATOR_INTERFACE_PWM_ACTUATOR_H_
#define COMMON_ACTUATOR_INTERFACE_PWM_ACTUATOR_H_

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

class PWMActuator : public Base<uint16_t>
{
public:
           PWMActuator(std::string const & name) : Base{std::string("[PWM Actuator] \"") + name + std::string("\"")} { }
  virtual ~PWMActuator() { }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

typedef std::shared_ptr<PWMActuator> SharedPWMActuator;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */

#endif /* COMMON_ACTUATOR_INTERFACE_PWM_ACTUATOR_H_ */
