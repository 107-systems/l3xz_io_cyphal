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

LegController::LegController(CanardNodeID const node_id,
                             phy::opencyphal::Node & node,
                             rclcpp::Logger const logger)
: OpenCyphalDevice{node_id, node, logger}
, _is_bumper_pressed{false}
, _femur_angle_deg{0.0f}
, _tibia_angle_deg{0.0f}
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

CanardNodeID LegController::toNodeId(Leg const leg)
{
  static std::map<Leg, CanardNodeID> const LEG_NODE_ID_MAP =
  {
    {Leg::LeftFront,   1},
    {Leg::LeftMiddle,  2},
    {Leg::LeftBack,    3},
    {Leg::RightBack,   4},
    {Leg::RightMiddle, 5},
    {Leg::RightFront,  6},
  };

  return LEG_NODE_ID_MAP.at(leg);
}

bool LegController::isBumperPressed()
{
  return _is_bumper_pressed;
}

float LegController::femurAngle_deg()
{
  return _femur_angle_deg;
}

float LegController::tibiaAngle_deg()
{
  return _tibia_angle_deg;
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool LegController::subscribeTibiaTipBumberMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalTibiaTipBumperMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      if (transfer.metadata.remote_node_id == node_id())
      {
        OpenCyphalTibiaTipBumperMessage const tibia_endpoint_switch = OpenCyphalTibiaTipBumperMessage::deserialize(transfer);
        _is_bumper_pressed = !tibia_endpoint_switch.data.value;
      }
    });
}

bool LegController::subscribeFemurAngleMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalFemurAnglePositionDegreeMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      if (transfer.metadata.remote_node_id == node_id())
      {
        OpenCyphalFemurAnglePositionDegreeMessage const as5048_a_angle = OpenCyphalFemurAnglePositionDegreeMessage::deserialize(transfer);
        _femur_angle_deg = as5048_a_angle.data.value;
      }
    });
}

bool LegController::subscribeTibiaAngleMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalTibiaAnglePositionDegreeMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      if (transfer.metadata.remote_node_id == node_id())
      {
        OpenCyphalTibiaAnglePositionDegreeMessage const as5048_b_angle = OpenCyphalTibiaAnglePositionDegreeMessage::deserialize(transfer);
        _tibia_angle_deg = as5048_b_angle.data.value;
      }
    });
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
