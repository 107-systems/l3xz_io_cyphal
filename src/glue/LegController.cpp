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
{
  if (!subscribeTibiaTipBumberMessage(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'OpenCyphalTibiaTipBumperMessage'");
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

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
