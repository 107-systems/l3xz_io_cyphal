/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <glue/HydraulicAnglePositionReader.h>

#include <glue/OpenCyphalMessageTypes.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

HydraulicAnglePositionReader::HydraulicAnglePositionReader(phy::opencyphal::Node & node,
                                                           rclcpp::Logger const logger)
: _mtx{}
, _leg_angle_position_map{}
{
  if (!subscribeFemurAngleMessage(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'OpenCyphalFemurAnglePositionDegreeMessage'");

  if (!subscribeTibiaAngleMessage(node))
    RCLCPP_ERROR(logger, "failed to subscribe to 'OpenCyphalTibiaAnglePositionDegreeMessage'");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::map<LegJointKey, float> HydraulicAnglePositionReader::doBulkRead()
{
  std::lock_guard<std::mutex> lock(_mtx);
  return _leg_angle_position_map;
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

LegJointKey HydraulicAnglePositionReader::femur_toLegJointKey(CanardNodeID const node_id)
{
  static std::map<CanardNodeID, LegJointKey> const NODE_ID_FEMUR_TO_LEG_JOINT_MAP =
  {
    {1, make_key(Leg::LeftFront,   Joint::Femur)},
    {2, make_key(Leg::LeftMiddle,  Joint::Femur)},
    {3, make_key(Leg::LeftBack,    Joint::Femur)},
    {4, make_key(Leg::RightBack,   Joint::Femur)},
    {5, make_key(Leg::RightMiddle, Joint::Femur)},
    {6, make_key(Leg::RightFront,  Joint::Femur)},
  };

  return NODE_ID_FEMUR_TO_LEG_JOINT_MAP.at(node_id);
}

LegJointKey HydraulicAnglePositionReader::tibia_toLegJointKey(CanardNodeID const node_id)
{
  static std::map<CanardNodeID, LegJointKey> const NODE_ID_TIBIA_TO_LEG_JOINT_MAP =
  {
    {1, make_key(Leg::LeftFront,   Joint::Tibia)},
    {2, make_key(Leg::LeftMiddle,  Joint::Tibia)},
    {3, make_key(Leg::LeftBack,    Joint::Tibia)},
    {4, make_key(Leg::RightBack,   Joint::Tibia)},
    {5, make_key(Leg::RightMiddle, Joint::Tibia)},
    {6, make_key(Leg::RightFront,  Joint::Tibia)},
  };

  return NODE_ID_TIBIA_TO_LEG_JOINT_MAP.at(node_id);
}

bool HydraulicAnglePositionReader::subscribeFemurAngleMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalFemurAnglePositionDegreeMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      OpenCyphalFemurAnglePositionDegreeMessage const as5048_a_angle = OpenCyphalFemurAnglePositionDegreeMessage::deserialize(transfer);
      LegJointKey const femur_key = femur_toLegJointKey(transfer.metadata.remote_node_id);

      std::lock_guard<std::mutex> lock(_mtx);
      _leg_angle_position_map[femur_key] = as5048_a_angle.data.value;
    });
}

bool HydraulicAnglePositionReader::subscribeTibiaAngleMessage(phy::opencyphal::Node & node)
{
  return node.subscribe<OpenCyphalTibiaAnglePositionDegreeMessage>(
    [this](CanardRxTransfer const & transfer)
    {
      OpenCyphalTibiaAnglePositionDegreeMessage const as5048_b_angle = OpenCyphalTibiaAnglePositionDegreeMessage::deserialize(transfer);
      LegJointKey const tibia_key = tibia_toLegJointKey(transfer.metadata.remote_node_id);

      std::lock_guard<std::mutex> lock(_mtx);
      _leg_angle_position_map[tibia_key] = as5048_b_angle.data.value;
    });
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
