/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef SSC32_SSC32_H_
#define SSC32_SSC32_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include <string>

#include <l3xz/phy/serial/AsyncSerial.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace ssc32
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

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

  Error setPulseWidth(uint8_t const channel, uint16_t const pulse_width_us, uint16_t const move_time_us);

private:
  phy::serial::AsyncSerial _serial;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* ssc32 */

#endif /* SSC32_SSC32_H_ */
