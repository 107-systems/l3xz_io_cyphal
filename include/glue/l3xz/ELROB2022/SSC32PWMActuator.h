/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/PWMActuator.h>

#include <driver/ssc32/SSC32.h>

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
  SSC32PWMActuator(std::string const & name, driver::SharedSSC32 ssc32, uint8_t const channel, uint16_t const initial_pulse_width_us)
  : PWMActuator(name)
  , _ssc32{ssc32}
  , _channel{channel}
  {
    set(initial_pulse_width_us);
  }
  virtual ~SSC32PWMActuator()
  { }

  virtual void set(uint16_t const & val) override
  {
    uint16_t pulse_width_us;

    if (val > 2000) pulse_width_us = 2000;
    if (val < 1000) pulse_width_us = 1000;
    else            pulse_width_us = val;

    _val = pulse_width_us;

    _ssc32->setPulseWidth(_channel, _val, 0);
  }


protected:
  virtual std::optional<uint16_t> get() const override
  {
    return _val;
  }


private:
  driver::SharedSSC32 _ssc32;
  uint8_t const _channel;
  uint16_t _val;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_H_ */
