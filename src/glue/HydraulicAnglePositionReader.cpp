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
{
  if (!node.subscribe<uavcan::primitive::scalar::Real32_1_0<1002>>(
    [this](CanardRxTransfer const & transfer)
    {
      uavcan::primitive::scalar::Real32_1_0<1002> const as5048_a_angle = uavcan::primitive::scalar::Real32_1_0<1002>::deserialize(transfer);
      this->update_femur_angle(transfer.metadata.remote_node_id, as5048_a_angle.data.value);
    }))
  {
    printf("[ERROR] HydraulicAnglePositionReader: failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1002>'");
  }

  if (!node.subscribe<uavcan::primitive::scalar::Real32_1_0<1003>>(
    [this](CanardRxTransfer const & transfer)
    {
      uavcan::primitive::scalar::Real32_1_0<1003> const as5048_b_angle = uavcan::primitive::scalar::Real32_1_0<1003>::deserialize(transfer);
      this->update_tibia_angle(transfer.metadata.remote_node_id, as5048_b_angle.data.value);
    }))
  {
    printf("[ERROR] HydraulicAnglePositionReader: failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1003>'");
  }
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void HydraulicAnglePositionReader::doBulkRead()
{
  std::lock_guard<std::mutex> lock(_mtx);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void HydraulicAnglePositionReader::update_femur_angle(CanardNodeID const node_id, float const femur_angle_deg)
{
  std::lock_guard<std::mutex> lock(_mtx);
}

void HydraulicAnglePositionReader::update_tibia_angle(CanardNodeID const node_id, float const tibia_angle_deg)
{
  std::lock_guard<std::mutex> lock(_mtx);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
