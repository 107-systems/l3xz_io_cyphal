/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdint>

#include <tuple>
#include <vector>
#include <memory>
#include <optional>

#include <dynamixel_sdk.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixel
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Error : int
{
  None          =  0,
  AddParam      = -1,
  TxPacket      = -2,
  TxRxPacket    = -3,
  BroadcastPing = -4,
};

typedef std::vector<uint8_t>                         IdVect;
typedef std::tuple<uint8_t, uint8_t *>               SyncWriteData;
typedef std::vector<SyncWriteData>                   SyncWriteDataVect;
typedef std::tuple<uint8_t, std::optional<uint32_t>> SyncReadData;
typedef std::vector<SyncReadData>                    SyncReadDataVect;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Dynamixel
{
public:

   Dynamixel(std::string const device_name,
             float       const protocol_version,
             int         const baudrate);
  ~Dynamixel();


  std::tuple<Error, IdVect> broadcastPing();

  Error syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteData const & data);
  Error syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteDataVect const & data);

  std::tuple<Error, SyncReadData>     syncRead(uint16_t const start_address, uint16_t const data_length, uint8_t const id);
  std::tuple<Error, SyncReadDataVect> syncRead(uint16_t const start_address, uint16_t const data_length, IdVect const & id_vect);


private:

  std::unique_ptr<PortHandler> _port_handler;
  std::unique_ptr<PacketHandler> _packet_handler;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

}; /* dynamixel */

#endif /* DYNAMIXEL_H_ */
