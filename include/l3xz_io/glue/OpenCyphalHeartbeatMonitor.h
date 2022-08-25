/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_OPEN_CYPHAL_HEARTBEAT_MONITOR_H_
#define GLUE_OPEN_CYPHAL_HEARTBEAT_MONITOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/phy/opencyphal/Node.hpp>
#include <l3xz_io/phy/opencyphal/Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class OpenCyphalHeartbeatMonitor
{
public:
  OpenCyphalHeartbeatMonitor(phy::opencyphal::Node & node, rclcpp::Logger const logger, std::list<CanardNodeID> const node_id_list)
  : _mtx{}
  {
    for (auto node_id : node_id_list)
      _node_heartbeat_data[node_id] = HeartbeatData();

    if (!subscribeHeartbeat(node))
      RCLCPP_ERROR(logger, "failed to subscribe to 'hearbeat'");
  }

  static std::string toStr(std::list<CanardNodeID> const & node_list)
  {
    std::stringstream ss;
    for (auto node_id : node_list)
      ss << static_cast<int>(node_id) << " ";
    return ss.str();
  }

  std::list<CanardNodeID> detectedNodeIdList()
  {
    std::lock_guard<std::mutex> lock(_mtx);
    return _detected_node_id_list;
  }

  std::tuple<bool, std::list<CanardNodeID>> isConnected(std::chrono::seconds const heartbeat_timeout)
  {
    std::lock_guard<std::mutex> lock(_mtx);

    bool is_connected = true;
    std::list<CanardNodeID> node_list;

    for (auto [node_id, data] : _node_heartbeat_data)
      if(!data.isConnected(heartbeat_timeout))
      {
        is_connected = false;
        node_list.push_back(node_id);
      }
    
    return std::tuple(is_connected, node_list);
  }

  std::tuple<bool, std::list<CanardNodeID>> isHealthy()
  {
    std::lock_guard<std::mutex> lock(_mtx);

    bool is_healty = true;
    std::list<CanardNodeID> node_list;

    for (auto [node_id, data] : _node_heartbeat_data)
      if(!data.isHealthy())
      {
        is_healty = false;
        node_list.push_back(node_id);
      }
    
    return std::tuple(is_healty, node_list);
  }

  std::tuple<bool, std::list<CanardNodeID>> isOperational()
  {
    std::lock_guard<std::mutex> lock(_mtx);

    bool is_operational = true;
    std::list<CanardNodeID> node_list;

    for (auto [node_id, data] : _node_heartbeat_data)
      if(!data.isOperational())
      {
        is_operational = false;
        node_list.push_back(node_id);
      }
    
    return std::tuple(is_operational, node_list);
  }

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

  bool subscribeHeartbeat(phy::opencyphal::Node & node)
  {
    return node.subscribe<uavcan::node::Heartbeat_1_0<>>(
        [this](CanardRxTransfer const & transfer)
        {
          uavcan::node::Heartbeat_1_0<> const heartbeat = uavcan::node::Heartbeat_1_0<>::deserialize(transfer);
            
          std::lock_guard<std::mutex> lock(_mtx);

          if (std::find(std::begin(_detected_node_id_list),
                        std::end  (_detected_node_id_list),
                        transfer.metadata.remote_node_id)
              == std::end(_detected_node_id_list))
          {
            _detected_node_id_list.push_back(transfer.metadata.remote_node_id);
          }

          HeartbeatData data(std::chrono::system_clock::now(), heartbeat.data.health, heartbeat.data.mode);
          _node_heartbeat_data[transfer.metadata.remote_node_id] = data;
        });
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_OPEN_CYPHAL_HEARTBEAT_MONITOR_H_ */
