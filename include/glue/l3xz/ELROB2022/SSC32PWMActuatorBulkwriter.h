/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_BULK_WRITER_H_
#define GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_BULK_WRITER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "SSC32PWMActuator.h"

#include <map>

#include <driver/ssc32/SSC32.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SSC32PWMActuatorBulkwriter
{
public:
  SSC32PWMActuatorBulkwriter(driver::SharedSSC32 ssc32)
  :_ssc32{ssc32}
  { }

  void update(uint8_t const channel, uint16_t const pulse_width_us)
  {
    _channel_pulse_width_map[channel] = pulse_width_us;
  }

  void doBulkWrite()
  {
    for (auto [channel, pulse_width_us] : _channel_pulse_width_map)
      _ssc32->setPulseWidth(channel, pulse_width_us, 50);
  }

private:
  driver::SharedSSC32 _ssc32;
  std::map<uint8_t, uint16_t> _channel_pulse_width_map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_SSC32_PWM_ACTUATOR_BULK_WRITER_H_ */
