/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef DYNAMIXEL_DYNAMIXEL_H_
#define DYNAMIXEL_DYNAMIXEL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdint>

#include <tuple>
#include <vector>

#include <dynamixel_sdk.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixel
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelController
{
public:

  DynamixelController(PortHandler & port_handler, PacketHandler & packet_handler);

  enum class Error : ssize_t
  {
    None     =  0,
    AddParam = -1,
    TxPacket = -2,
  };

  typedef std::tuple<uint8_t, uint8_t *> SyncWriteData;
  Error syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteData const & data);
  Error syncWrite(uint16_t const start_address, uint16_t const data_length, std::vector<SyncWriteData> const & data);


private:

  PortHandler & _port_handler;
  PacketHandler & _packet_handler;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

}; /* dynamixel */

#endif /* DYNAMIXEL_DYNAMIXEL_H_ */
