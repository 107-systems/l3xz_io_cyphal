/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef DRIVER_SSC32_SSC32_H_
#define DRIVER_SSC32_SSC32_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include <string>
#include <memory>

#include <l3xz_ros_cyphal_bridge/phy/serial/AsyncSerial.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SSC32
{
public:
   SSC32(std::string const device_name, size_t const baudrate);
  ~SSC32();

  static uint8_t constexpr MIN_CHANNEL =  0;
  static uint8_t constexpr MAX_CHANNEL = 31;

  void setPulseWidth(uint8_t const channel, uint16_t const pulse_width_us, uint16_t const move_time_ms);

private:
  phy::serial::AsyncSerial _serial;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<SSC32> SharedSSC32;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* DRIVER_SSC32_SSC32_H_ */
