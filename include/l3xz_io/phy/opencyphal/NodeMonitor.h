/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef PHY_OPENCYPHAL_open_cyphal_node_monitor_H_
#define PHY_OPENCYPHAL_open_cyphal_node_monitor_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/phy/opencyphal/Types.h>
#include <l3xz_io/phy/opencyphal/Node.hpp>

#include <rclcpp/rclcpp.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace phy::opencyphal
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class NodeMonitor
{
public:
  NodeMonitor(Node & node, rclcpp::Logger const logger, std::list<CanardNodeID> const node_id_list);

  static std::string toStr(std::list<CanardNodeID> const & node_list);
  std::list<CanardNodeID> detectedNodeIdList();

  std::tuple<bool, std::list<CanardNodeID>> isConnected(std::chrono::seconds const heartbeat_timeout);
  std::tuple<bool, std::list<CanardNodeID>> isHealthy();
  std::tuple<bool, std::list<CanardNodeID>> isOperational();

private:

  class HeartbeatData
  {
  private:
    static uavcan_node_Health_1_0 constexpr INITAL_HEARTBEAT_HEALTH_DATA = { uavcan_node_Health_1_0_WARNING };
    static uavcan_node_Mode_1_0   constexpr INITIAL_HEARTBEAT_MODE_DATA = { uavcan_node_Mode_1_0_INITIALIZATION };
    std::chrono::system_clock::time_point _timestamp;
    uavcan_node_Health_1_0 _health;
    uavcan_node_Mode_1_0 _mode;
  public:
    HeartbeatData(std::chrono::system_clock::time_point const timestamp, uavcan_node_Health_1_0 const health, uavcan_node_Mode_1_0 const mode)
    : _timestamp(timestamp)
    , _health(health)
    , _mode(mode)
    { }
    HeartbeatData() : HeartbeatData(std::chrono::system_clock::now(),INITAL_HEARTBEAT_HEALTH_DATA, INITIAL_HEARTBEAT_MODE_DATA)
    { }
    inline bool isConnected(std::chrono::seconds const heartbeat_timeout) { return ((std::chrono::system_clock::now() - _timestamp) < heartbeat_timeout); }
    inline bool isHealthy() { return _health.value == uavcan_node_Health_1_0_NOMINAL; }
    inline bool isOperational() { return _mode.value == uavcan_node_Mode_1_0_OPERATIONAL; }
  };

  std::mutex _mtx;
  std::map<CanardNodeID, HeartbeatData> _node_heartbeat_data;
  std::list<CanardNodeID> _detected_node_id_list;

  bool subscribeHeartbeat(Node & node);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* phy::opencyphal */

#endif /* PHY_OPENCYPHAL_open_cyphal_node_monitor_H_ */
