/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <glue/HydraulicAnglePositionReader.h>

#include <phy/opencyphal/Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

HydraulicAnglePositionReader::HydraulicAnglePositionReader(phy::opencyphal::Node & node)
: _mtx{}
, _leg_angle_position_map{}
{
  if (!node.subscribe<uavcan::primitive::scalar::Real32_1_0<1002>>(
    [this](CanardRxTransfer const & transfer)
    {
      uavcan::primitive::scalar::Real32_1_0<1002> const as5048_a_angle = uavcan::primitive::scalar::Real32_1_0<1002>::deserialize(transfer);
      LegJointKey const femur_key = femur_toLegJointKey(transfer.metadata.remote_node_id);

      std::lock_guard<std::mutex> lock(_mtx);
      _leg_angle_position_map[femur_key] = as5048_a_angle.data.value;
    }))
  {
    printf("[ERROR] HydraulicAnglePositionReader: failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1002>'");
  }

  if (!node.subscribe<uavcan::primitive::scalar::Real32_1_0<1003>>(
    [this](CanardRxTransfer const & transfer)
    {
      uavcan::primitive::scalar::Real32_1_0<1003> const as5048_b_angle = uavcan::primitive::scalar::Real32_1_0<1003>::deserialize(transfer);
      LegJointKey const tibia_key = tibia_toLegJointKey(transfer.metadata.remote_node_id);

      std::lock_guard<std::mutex> lock(_mtx);
      _leg_angle_position_map[tibia_key] = as5048_b_angle.data.value;
    }))
  {
    printf("[ERROR] HydraulicAnglePositionReader: failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1003>'");
  }
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

  return NODE_ID_TIBIA_TO_LEG_JOINT_MAP.at(node_id);}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
