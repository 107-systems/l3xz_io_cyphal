/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <glue/BumperSensorReader.h>

#include <glue/OpenCyphalMessageTypes.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

BumperSensorReader::BumperSensorReader(phy::opencyphal::Node & node,
                                       rclcpp::Logger const logger)
: _mtx{}
, _tibia_tip_bumper_map
  {
    {Leg::LeftFront,   false},
    {Leg::LeftMiddle,  false},
    {Leg::LeftBack,    false},
    {Leg::RightBack,   false},
    {Leg::RightMiddle, false},
    {Leg::RightFront,  false},
  }
{
  if (!subscribeTibiaTipBumberMessage(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'OpenCyphalTibiaTipBumperMessage'");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::map<Leg, bool> BumperSensorReader::doBulkRead()
{
  std::lock_guard<std::mutex> lock(_mtx);
  return _tibia_tip_bumper_map;
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

Leg BumperSensorReader::toLeg(CanardNodeID const node_id)
{
  static std::map<CanardNodeID, Leg> const NODE_ID_LEG_MAP =
  {
    {1, Leg::LeftFront},
    {2, Leg::LeftMiddle},
    {3, Leg::LeftBack},
    {4, Leg::RightBack},
    {5, Leg::RightMiddle},
    {6, Leg::RightFront},
  };

  return NODE_ID_LEG_MAP.at(node_id);
}

bool BumperSensorReader::subscribeTibiaTipBumberMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalTibiaTipBumperMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      OpenCyphalTibiaTipBumperMessage const tibia_endpoint_switch = OpenCyphalTibiaTipBumperMessage::deserialize(transfer);
      Leg const leg = toLeg(transfer.metadata.remote_node_id);
      bool const is_pressed = !tibia_endpoint_switch.data.value;

      std::lock_guard<std::mutex> lock(_mtx);
      _tibia_tip_bumper_map[leg] = is_pressed;
    });
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
