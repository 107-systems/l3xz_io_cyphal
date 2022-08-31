/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_io/control/ssc32/SSC32.h>

#include <vector>
#include <sstream>
#include <stdexcept>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

SSC32::SSC32(std::string const device_name, size_t const baudrate)
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

void SSC32::setPulseWidth(uint8_t const channel, uint16_t const pulse_width_us, uint16_t const move_time_ms)
{
  auto isValidChannel    = [](size_t const c) -> bool { return ((c >= 0) && (c <= 31)); };
  auto isValidPulseWidth = [](size_t const p) -> bool { return ((p >= 500) && (p <= 2500)); };

  if (!isValidChannel(channel))
  {
    std::stringstream err_msg;
    err_msg << "SSC32::setPulseWidth: error invalid channel parameter: "
            << static_cast<int>(channel);
    throw std::runtime_error(err_msg.str());
  }

  if (!isValidPulseWidth(pulse_width_us))
  {
    std::stringstream err_msg;
    err_msg << "SSC32::setPulseWidth: error invalid pulse width parameter: "
            << static_cast<int>(pulse_width_us);
    throw std::runtime_error(err_msg.str());
  }

  std::stringstream msg;
  msg << "#"
      << static_cast<unsigned int>(channel)
      << "P"
      << static_cast<unsigned int>(pulse_width_us)
      << "T"
      << static_cast<unsigned int>(move_time_ms)
      << '\r';
  std::string const msg_str(msg.str());
  std::vector<uint8_t> const msg_vect(msg_str.begin(), msg_str.end());

  _serial.transmit(msg_vect);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */
