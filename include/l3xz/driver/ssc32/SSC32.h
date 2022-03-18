/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef DRIVER_SSC32_SSC32_H_
#define DRIVER_SSC32_SSC32_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include <string>

#include <l3xz/phy/serial/AsyncSerial.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::driver
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SSC32
{
public:
   SSC32(std::string const device_name, size_t const baudrate);
  ~SSC32();

  enum class Error : int
  {
    None                =  0,
    InvParam_Channel    = -1,
    InvParam_PulseWidth = -2,
  };

  Error setPulseWidth(uint8_t const channel, uint16_t const pulse_width_us, uint16_t const move_time_ms);

private:
  phy::serial::AsyncSerial _serial;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::driver */

#endif /* DRIVER_SSC32_SSC32_H_ */
