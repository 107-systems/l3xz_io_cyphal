/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/ValveActuator.h>

#include <common/actuator/interface/PWMActuator.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SSC32ValveActuator : public common::actuator::interface::ValveActuator
{
public:
  SSC32ValveActuator(std::string const & name, common::actuator::interface::SharedPWMActuator pwm_actuator)
  : ValveActuator(name)
  , _pwm_actuator{pwm_actuator}
  { }
  virtual ~SSC32ValveActuator() { }

  virtual void set(float const & val) override
  {
    float limited_val = val;
    if (limited_val >  1.0f) limited_val =  1.0f;
    if (limited_val < -1.0f) limited_val = -1.0f;

    _val = limited_val;

    float const pulse_width_us = (_val.value() * 500) + 1500;

    _pwm_actuator->set(pulse_width_us);
  }

protected:
  virtual std::optional<float> get() const override
  {
    return _val;
  }

private:
  common::actuator::interface::SharedPWMActuator _pwm_actuator;
  std::optional<float> _val;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_ */
