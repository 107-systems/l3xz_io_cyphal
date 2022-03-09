/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz/dynamixel/DynamixelController.h>

#include <ros/console.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixel
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

DynamixelController::DynamixelController(PortHandler & port_handler, PacketHandler & packet_handler)
: _port_handler{port_handler}
, _packet_handler{packet_handler}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTION
 **************************************************************************************/

DynamixelController::Error DynamixelController::syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteData const & data)
{
  return syncWrite(start_address, data_length, std::vector<SyncWriteData>{data});
}

DynamixelController::Error DynamixelController::syncWrite(uint16_t const start_address, uint16_t const data_length, std::vector<SyncWriteData> const & data)
{
  GroupSyncWrite group_sync_write(&_port_handler, &_packet_handler, start_address, data_length);

  for(auto [id, data_ptr] : data)
  {
    if (!group_sync_write.addParam(id, data_ptr))
      return Error::AddParam;
  }

  if (int res = group_sync_write.txPacket(); res != COMM_SUCCESS)
  {
    ROS_ERROR("%s", _packet_handler.getTxRxResult(res));
    return Error::TxPacket;
  }

  group_sync_write.clearParam();

  return Error::None;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

}; /* dynamixel */
