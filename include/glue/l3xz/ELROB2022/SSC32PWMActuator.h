/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/PWMActuator.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SSC32PWMActuator : public common::actuator::interface::PWMActuator
{
public:
  typedef std::function<void(uint8_t const, uint16_t const)> OnChangeFunc;

  SSC32PWMActuator(std::string const & name, uint8_t const channel, OnChangeFunc func)
  : PWMActuator(name)
  , _channel{channel}
  , _on_change_func{func}
  { }
  virtual ~SSC32PWMActuator() { }

  virtual void set(uint16_t const & val) override
  {
    uint16_t pulse_width_us;

    if (val > 2000) pulse_width_us = 2000;
    if (val < 1000) pulse_width_us = 1000;
    else            pulse_width_us = val;

    _on_change_func(_channel, pulse_width_us);

    _pulse_width_us = pulse_width_us;
  }


protected:
  virtual std::optional<uint16_t> get() const override
  {
    return _pulse_width_us;
  }

private:
  uint8_t const _channel;
  OnChangeFunc _on_change_func;
  std::optional<uint16_t> _pulse_width_us;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_H_ */
