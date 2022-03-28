/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef VALVE_H_
#define VALVE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/Const.h>
#include <l3xz/driver/ssc32/SSC32.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Valve
{
public:
  Valve(driver::SharedSSC32 & ssc32);

  void set(Leg const leg, Joint const joint, uint16_t const pulse_width_us);

private:
  driver::SharedSSC32 _ssc32;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* VALVE_H_ */
