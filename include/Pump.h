/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef PUMP_H_
#define PUMP_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <driver/ssc32/SSC32.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Pump
{
public:
  Pump(driver::SharedSSC32 & ssc32);

  void set(uint16_t const pulse_width_us);

private:
  driver::SharedSSC32 _ssc32;

  static uint8_t constexpr PUMP_CHANNEL = 15;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* PUMP_H_ */
