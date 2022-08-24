/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/glue/OpenCyphalDevice.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

OpenCyphalDevice::OpenCyphalDevice(CanardNodeID const node_id,
                                   phy::opencyphal::Node & node,
                                   rclcpp::Logger const logger)
: _node_id{node_id}
{
  if (!subscribeHeartbeat(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'hearbeat'");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool OpenCyphalDevice::isHeartbeatTimeout(std::chrono::seconds const timeout)
{
  std::chrono::system_clock::time_point last_receive_timestamp;

  {
    std::lock_guard<std::mutex> lock(_heartbeat);
    last_receive_timestamp = _heartbeat.receive_timestamp;
  }

  auto const duration_since_last_receive_timestamp = (std::chrono::system_clock::now() - last_receive_timestamp);
  if (duration_since_last_receive_timestamp > timeout)
    return true;
  else
    return false;
}

bool OpenCyphalDevice::isModeOperational()
{
  std::lock_guard<std::mutex> lock(_heartbeat);
  return (_heartbeat.data.mode.value == uavcan_node_Mode_1_0_OPERATIONAL);
}

bool OpenCyphalDevice::isHealthNominal()
{
  std::lock_guard<std::mutex> lock(_heartbeat);
  return (_heartbeat.data.health.value == uavcan_node_Health_1_0_NOMINAL);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool OpenCyphalDevice::subscribeHeartbeat(phy::opencyphal::Node & node)
{
  return node.subscribe<uavcan::node::Heartbeat_1_0<>>(
    [this](CanardRxTransfer const & transfer)
    {
      if (transfer.metadata.remote_node_id == _node_id)
      {
        uavcan::node::Heartbeat_1_0<> const heartbeat = uavcan::node::Heartbeat_1_0<>::deserialize(transfer);
        std::lock_guard<std::mutex> lock(_heartbeat);
        _heartbeat.data = heartbeat.data;
        _heartbeat.receive_timestamp = std::chrono::system_clock::now();
      }
    });
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
