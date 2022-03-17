/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz/driver/ssc32/SSC32.h>

#include <vector>
#include <sstream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::driver
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

SSC32::SSC32(std::string const device_name,
             size_t const baudrate)
{
  _serial.open(device_name, baudrate);
}

SSC32::~SSC32()
{
  _serial.close();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

SSC32::Error SSC32::setPulseWidth(uint8_t const channel, uint16_t const pulse_width_us, uint16_t const move_time_us)
{
  auto isValidChannel    = [](size_t const c) -> bool { return ((c >= 0) && (c <= 31)); };
  auto isValidPulseWidth = [](size_t const p) -> bool { return ((p >= 500) && (p <= 2500)); };

  if (!isValidChannel(channel))
    return Error::InvParam_Channel;

  if (!isValidPulseWidth(pulse_width_us))
    return Error::InvParam_PulseWidth;

  std::stringstream msg;
  msg << "#"
      << static_cast<unsigned int>(channel)
      << "P"
      << static_cast<unsigned int>(pulse_width_us)
      << "T"
      << static_cast<unsigned int>(move_time_us)
      << '\r';
  std::string const msg_str(msg.str());
  std::vector<uint8_t> const msg_vect(msg_str.begin(), msg_str.end());

  _serial.transmit(msg_vect);

  return Error::None;
}


/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::driver */
