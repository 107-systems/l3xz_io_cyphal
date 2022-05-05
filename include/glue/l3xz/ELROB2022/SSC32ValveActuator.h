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

#include <driver/ssc32/SSC32.h>

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
  SSC32ValveActuator(std::string const & name, driver::SharedSSC32 ssc32, uint8_t const channel, float const initial_val)
  : ValveActuator(name)
  , _ssc32{ssc32}
  , _channel{channel}
  {
    set(initial_val);
  }
  virtual ~SSC32ValveActuator()
  { }

  virtual void set(float const val) override
  {
    float limited_val = val;
    if (limited_val >  1.0f) limited_val =  1.0f;
    if (limited_val < -1.0f) limited_val = -1.0f;

    _val = limited_val;

    float const pulse_width_us = (limited_val * 500) + 1500;

    _ssc32->setPulseWidth(_channel, static_cast<uint16_t>(pulse_width_us), 0);
  }

  virtual std::string toStr() const override
  {
    std::stringstream ss;
    ss << "[A] " << name() << ": " << _val;
    return ss.str();
  }

private:
  driver::SharedSSC32 _ssc32;
  uint8_t const _channel;
  float _val;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_ */
