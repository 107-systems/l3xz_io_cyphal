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

DynamixelController::DynamixelController(std::string const device_name,
                                         float       const protocol_version,
                                         int         const baudrate)
: _port_handler{PortHandler::getPortHandler(device_name.c_str())}
, _packet_handler{PacketHandler::getPacketHandler(protocol_version)}
{
  if (!_port_handler->openPort())
    ROS_FATAL("%s::%s error, 'PortHandler::openPort()' failed.", __FILE__, __FUNCTION__);

  if (!_port_handler->setBaudRate(baudrate))
    ROS_FATAL("%s::%s error, 'PortHandler::setBaudRate()' failed.", __FILE__, __FUNCTION__);
}

DynamixelController::~DynamixelController()
{
  _port_handler->closePort();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTION
 **************************************************************************************/

std::tuple<Error, std::vector<uint8_t>> DynamixelController::broadcastPing()
{
  std::vector<uint8_t> servo_id_vect;

  if (int const res = _packet_handler->broadcastPing(_port_handler.get(), servo_id_vect); res != COMM_SUCCESS)
  {
    ROS_ERROR("%s::%s error, 'PacketHandler::broadcastPing()' failed.", __FILE__, __FUNCTION__);
    return std::make_tuple(Error::BroadcastPing, servo_id_vect);
  }

  return std::make_tuple(Error::None, servo_id_vect);
}

Error DynamixelController::syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteData const & data)
{
  return syncWrite(start_address, data_length, std::vector<SyncWriteData>{data});
}

Error DynamixelController::syncWrite(uint16_t const start_address, uint16_t const data_length, std::vector<SyncWriteData> const & data)
{
  GroupSyncWrite group_sync_write(_port_handler.get(), _packet_handler.get(), start_address, data_length);

  for(auto [id, data_ptr] : data)
  {
    if (!group_sync_write.addParam(id, data_ptr))
      return Error::AddParam;
  }

  if (int res = group_sync_write.txPacket(); res != COMM_SUCCESS)
  {
    ROS_ERROR("%s::%s error, 'GroupSyncWrite::txPacket()' %s", __FILE__, __FUNCTION__, _packet_handler->getTxRxResult(res));
    return Error::TxPacket;
  }

  group_sync_write.clearParam();

  return Error::None;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

}; /* dynamixel */
