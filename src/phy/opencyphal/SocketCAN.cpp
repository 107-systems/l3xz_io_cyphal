/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <phy/opencyphal/SocketCAN.h>

#include <stdexcept>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace phy::opencyphal
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

SocketCAN::SocketCAN(std::string const & iface_name, bool const is_can_fd)
: _fd{socketcanOpen(iface_name.c_str(), is_can_fd)}
{
  if (_fd < 0)
    throw std::runtime_error("SocketCAN::SocketCAN error opening CAN interface '" + iface_name + std::string("'"));
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

int16_t SocketCAN::filter(const size_t num_configs, const SocketCANFilterConfig * const configs)
{
  return socketcanFilter(_fd, num_configs, configs);
}

int16_t SocketCAN::push(const CanardFrame * const frame, const CanardMicrosecond timeout_usec)
{
  return socketcanPush(_fd, frame, timeout_usec);
}

int16_t SocketCAN::pop(CanardFrame * const      out_frame,
                       const  size_t            payload_buffer_size,
                       void * const             payload_buffer,
                       const  CanardMicrosecond timeout_usec,
                       bool * const             loopback)
{
  return socketcanPop(_fd, out_frame, payload_buffer_size, payload_buffer, timeout_usec, loopback);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* phy::opencyphal */
