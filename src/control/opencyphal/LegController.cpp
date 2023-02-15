/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_ros_cyphal_bridge/control/opencyphal/LegController.h>

#include <l3xz_ros_cyphal_bridge/control/opencyphal/OpenCyphalMessageTypes.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

LegController::LegController(phy::opencyphal::Node & node, rclcpp::Logger const logger)
: _mtx{}
, _is_bumper_pressed
  {
    {Leg::LeftFront  , false},
    {Leg::LeftMiddle , false},
    {Leg::LeftBack   , false},
    {Leg::RightBack  , false},
    {Leg::RightMiddle, false},
    {Leg::RightFront , false},
  }
, _femur_angle_deg
  {
    {Leg::LeftFront  , 0.0f},
    {Leg::LeftMiddle , 0.0f},
    {Leg::LeftBack   , 0.0f},
    {Leg::RightBack  , 0.0f},
    {Leg::RightMiddle, 0.0f},
    {Leg::RightFront , 0.0f},
  }
, _tibia_angle_deg
  {
    {Leg::LeftFront  , 0.0f},
    {Leg::LeftMiddle , 0.0f},
    {Leg::LeftBack   , 0.0f},
    {Leg::RightBack  , 0.0f},
    {Leg::RightMiddle, 0.0f},
    {Leg::RightFront , 0.0f},
  }
{
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

} /* control */
