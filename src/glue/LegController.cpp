/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/glue/LegController.h>

#include <l3xz_io/glue/OpenCyphalMessageTypes.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

LegController::LegController(phy::opencyphal::Node & node, rclcpp::Logger const logger)
: _mtx{}
{
  _is_bumper_pressed[Leg::LeftFront  ] = false;
  _is_bumper_pressed[Leg::LeftMiddle ] = false;
  _is_bumper_pressed[Leg::LeftBack   ] = false;
  _is_bumper_pressed[Leg::RightBack  ] = false;
  _is_bumper_pressed[Leg::RightMiddle] = false;
  _is_bumper_pressed[Leg::RightFront ] = false;

  _femur_angle_deg[Leg::LeftFront  ] = 0.0f;
  _femur_angle_deg[Leg::LeftMiddle ] = 0.0f;
  _femur_angle_deg[Leg::LeftBack   ] = 0.0f;
  _femur_angle_deg[Leg::RightBack  ] = 0.0f;
  _femur_angle_deg[Leg::RightMiddle] = 0.0f;
  _femur_angle_deg[Leg::RightFront ] = 0.0f;

  _tibia_angle_deg[Leg::LeftFront  ] = 0.0f;
  _tibia_angle_deg[Leg::LeftMiddle ] = 0.0f;
  _tibia_angle_deg[Leg::LeftBack   ] = 0.0f;
  _tibia_angle_deg[Leg::RightBack  ] = 0.0f;
  _tibia_angle_deg[Leg::RightMiddle] = 0.0f;
  _tibia_angle_deg[Leg::RightFront ] = 0.0f;

  uavcan_node_Health_1_0 const INITAL_HEARTBEAT_HEALTH_DATA = { uavcan_node_Health_1_0_WARNING };
  uavcan_node_Mode_1_0 const INITIAL_HEARTBEAT_MODE_DATA = { uavcan_node_Mode_1_0_INITIALIZATION };

  THeartbeatData const INITAL_HEARTBEAT_DATA =
  {
    INITAL_HEARTBEAT_HEALTH_DATA, INITIAL_HEARTBEAT_MODE_DATA, std::chrono::system_clock::now()
  };

  _heartbeat[Leg::LeftFront  ] = INITAL_HEARTBEAT_DATA;
  _heartbeat[Leg::LeftMiddle ] = INITAL_HEARTBEAT_DATA;
  _heartbeat[Leg::LeftBack   ] = INITAL_HEARTBEAT_DATA;
  _heartbeat[Leg::RightBack  ] = INITAL_HEARTBEAT_DATA;
  _heartbeat[Leg::RightMiddle] = INITAL_HEARTBEAT_DATA;
  _heartbeat[Leg::RightFront ] = INITAL_HEARTBEAT_DATA;


  if (!subscribeHeartbeat(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'hearbeat'");

  if (!subscribeTibiaTipBumberMessage(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'OpenCyphalTibiaTipBumperMessage'");

  if (!subscribeFemurAngleMessage(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'OpenCyphalFemurAnglePositionDegreeMessage'");

  if (!subscribeTibiaAngleMessage(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'OpenCyphalTibiaAnglePositionDegreeMessage'");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool LegController::isHeartbeatTimeout(Leg const leg, std::chrono::seconds const timeout)
{
  std::lock_guard<std::mutex> lock(_mtx);

  auto const duration_since_last_receive_timestamp = (std::chrono::system_clock::now() - _heartbeat.at(leg).timestamp);
  if (duration_since_last_receive_timestamp > timeout)
    return true;
  else
    return false;
}

bool LegController::isModeOperational(Leg const leg)
{
  std::lock_guard<std::mutex> lock(_mtx);
  return (_heartbeat.at(leg).mode.value == uavcan_node_Mode_1_0_OPERATIONAL);
}

bool LegController::isHealthNominal(Leg const leg)
{
  std::lock_guard<std::mutex> lock(_mtx);
  return (_heartbeat.at(leg).health.value == uavcan_node_Health_1_0_NOMINAL);
}

bool LegController::isBumperPressed(Leg const leg)
{
  std::lock_guard<std::mutex> lock(_mtx);
  return _is_bumper_pressed.at(leg);
}

float LegController::femurAngle_deg(Leg const leg)
{
  std::lock_guard<std::mutex> lock(_mtx);
  return _femur_angle_deg.at(leg);
}

float LegController::tibiaAngle_deg(Leg const leg)
{
  std::lock_guard<std::mutex> lock(_mtx);
  return _tibia_angle_deg.at(leg);
}

CanardNodeID LegController::toNodeId(Leg const leg)
{
  static std::map<Leg, CanardNodeID> LEG_2_NODE_ID_MAP =
  {
    {Leg::LeftFront,   1},
    {Leg::LeftMiddle,  2},
    {Leg::LeftBack,    3},
    {Leg::RightBack,   4},
    {Leg::RightMiddle, 5},
    {Leg::RightFront,  6},
  };

  return LEG_2_NODE_ID_MAP.at(leg);
}

Leg LegController::toLeg(CanardNodeID const node_id)
{
  static std::map<CanardNodeID, Leg> const NODE_ID_2_LEG_MAP =
  {
    {1, Leg::LeftFront},
    {2, Leg::LeftMiddle},
    {3, Leg::LeftBack},
    {4, Leg::RightBack},
    {5, Leg::RightMiddle},
    {6, Leg::RightFront},
  };

  return NODE_ID_2_LEG_MAP.at(node_id);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool LegController::subscribeHeartbeat(phy::opencyphal::Node & node)
{
  return node.subscribe<uavcan::node::Heartbeat_1_0<>>(
    [this](CanardRxTransfer const & transfer)
    {
      if (isLegControllerId(transfer.metadata.remote_node_id))
      {
        uavcan::node::Heartbeat_1_0<> const heartbeat = uavcan::node::Heartbeat_1_0<>::deserialize(transfer);
        
        std::lock_guard<std::mutex> lock(_mtx);

        THeartbeatData data;
        data.health    = heartbeat.data.health;
        data.mode      = heartbeat.data.mode;
        data.timestamp = std::chrono::system_clock::now();

        _heartbeat[toLeg(transfer.metadata.remote_node_id)] = data;
      }
    });
}

bool LegController::subscribeTibiaTipBumberMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalTibiaTipBumperMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      if (isLegControllerId(transfer.metadata.remote_node_id))
      {
        OpenCyphalTibiaTipBumperMessage const tibia_endpoint_switch = OpenCyphalTibiaTipBumperMessage::deserialize(transfer);
        
        std::lock_guard<std::mutex> lock(_mtx);
        _is_bumper_pressed[toLeg(transfer.metadata.remote_node_id)] = !tibia_endpoint_switch.data.value;
      }
    });
}

bool LegController::subscribeFemurAngleMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalFemurAnglePositionDegreeMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      if (isLegControllerId(transfer.metadata.remote_node_id))
      {
        OpenCyphalFemurAnglePositionDegreeMessage const as5048_a_angle = OpenCyphalFemurAnglePositionDegreeMessage::deserialize(transfer);

        std::lock_guard<std::mutex> lock(_mtx);
        _femur_angle_deg[toLeg(transfer.metadata.remote_node_id)] = as5048_a_angle.data.value;
      }
    });
}

bool LegController::subscribeTibiaAngleMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalTibiaAnglePositionDegreeMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      if (isLegControllerId(transfer.metadata.remote_node_id))
      {
        OpenCyphalTibiaAnglePositionDegreeMessage const as5048_b_angle = OpenCyphalTibiaAnglePositionDegreeMessage::deserialize(transfer);

        std::lock_guard<std::mutex> lock(_mtx);
        _tibia_angle_deg[toLeg(transfer.metadata.remote_node_id)] = as5048_b_angle.data.value;
      }
    });
}

bool LegController::isLegControllerId(CanardNodeID const node_id)
{
  static std::vector<CanardNodeID> const LEG_CONTROLLED_NODE_ID_LIST = {1, 2, 3, 4, 5, 6};
  return std::find(std::begin(LEG_CONTROLLED_NODE_ID_LIST), std::end(LEG_CONTROLLED_NODE_ID_LIST), node_id) != std::end(LEG_CONTROLLED_NODE_ID_LIST);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
