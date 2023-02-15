/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_ros_cyphal_bridge/phy/opencyphal/NodeMonitor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace phy::opencyphal
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

NodeMonitor::NodeMonitor(Node & node, rclcpp::Logger const logger, std::list<CanardNodeID> const node_id_list)
: _mtx{}
{
  for (auto node_id : node_id_list)
    _node_heartbeat_data[node_id] = HeartbeatData();

  if (!subscribeHeartbeat(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'hearbeat'");
}

std::string NodeMonitor::toStr(std::list<CanardNodeID> const & node_list)
{
  std::stringstream ss;
  for (auto node_id : node_list)
    ss << static_cast<int>(node_id) << " ";
  return ss.str();
}

std::list<CanardNodeID> NodeMonitor::detectedNodeIdList()
{
  std::lock_guard<std::mutex> lock(_mtx);
  return _detected_node_id_list;
}

std::tuple<bool, std::list<CanardNodeID>> NodeMonitor::isConnected(std::chrono::seconds const heartbeat_timeout)
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

std::tuple<bool, std::list<CanardNodeID>> NodeMonitor::isHealthy()
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

std::tuple<bool, std::list<CanardNodeID>> NodeMonitor::isOperational()
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

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool NodeMonitor::subscribeHeartbeat(Node & node)
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
          _detected_node_id_list.sort(); /* Make sure its in ascending order for better readability. */
        }

        HeartbeatData data(std::chrono::system_clock::now(), heartbeat.data.health, heartbeat.data.mode);
        _node_heartbeat_data[transfer.metadata.remote_node_id] = data;
      });
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* phy::opencyphal */
