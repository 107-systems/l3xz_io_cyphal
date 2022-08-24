/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_OPEN_CYPHAL_DEVICE_H_
#define GLUE_OPEN_CYPHAL_DEVICE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <mutex>
#include <chrono>

#include <l3xz_io/phy/opencyphal/Types.h>
#include <l3xz_io/phy/opencyphal/Node.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class OpenCyphalDevice
{
public:
  OpenCyphalDevice(CanardNodeID const node_id,
                   phy::opencyphal::Node & node,
                   rclcpp::Logger const logger);

  bool isHeartbeatTimeout(std::chrono::seconds const timeout);
  bool isModeOperational();
  bool isHealthNominal();

  inline CanardNodeID node_id() const { return _node_id; };

private:
  CanardNodeID _node_id;

  struct threadsafe_uavcan_node_Heartbeat_1_0 : public std::mutex {
    uavcan_node_Heartbeat_1_0 data;
    std::chrono::system_clock::time_point receive_timestamp;
  } _heartbeat;

  bool subscribeHeartbeat(phy::opencyphal::Node & node);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_OPEN_CYPHAL_DEVICE_H_ */
