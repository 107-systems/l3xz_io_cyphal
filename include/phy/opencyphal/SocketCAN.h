/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef PHY_OPENCYPHAL_SOCKETCAN_H_
#define PHY_OPENCYPHAL_SOCKETCAN_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>

#include <socketcan.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace phy::opencyphal
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SocketCAN
{
public:
  SocketCAN(std::string const & iface_name, bool const is_can_fd);


  int16_t filter(const size_t num_configs, const SocketCANFilterConfig * const configs);

  int16_t push(const CanardFrame * const frame, const CanardMicrosecond timeout_usec);
  int16_t pop (CanardFrame * const      out_frame,
               const  size_t            payload_buffer_size,
               void * const             payload_buffer,
               const  CanardMicrosecond timeout_usec,
               bool * const             loopback);

private:
  SocketCANFD _fd;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* phy::opencyphal */

#endif /* PHY_OPENCYPHAL_SOCKETCAN_H_ */
