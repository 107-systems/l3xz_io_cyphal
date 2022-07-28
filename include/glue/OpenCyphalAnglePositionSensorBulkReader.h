/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_
#define GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <mutex>

#include <phy/opencyphal/Node.hpp>
#include <phy/opencyphal/Types.h>

#include <glue/OpenCyphalNodeIdList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class OpenCyphalAnglePositionSensorBulkReader
{
public:
  OpenCyphalAnglePositionSensorBulkReader(phy::opencyphal::Node & node)
  {
    if (!node.subscribe<uavcan::primitive::scalar::Real32_1_0<1002>>(
      [this](CanardRxTransfer const & transfer)
      {
        uavcan::primitive::scalar::Real32_1_0<1002> const as5048_a_angle = uavcan::primitive::scalar::Real32_1_0<1002>::deserialize(transfer);
        this->update_femur_angle(transfer.metadata.remote_node_id, as5048_a_angle.data.value);
      }))
    {
      printf("[ERROR] OpenCyphalAnglePositionSensorBulkReader: failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1002>'");
    }

    if (!node.subscribe<uavcan::primitive::scalar::Real32_1_0<1003>>(
      [this](CanardRxTransfer const & transfer)
      {
        uavcan::primitive::scalar::Real32_1_0<1003> const as5048_b_angle = uavcan::primitive::scalar::Real32_1_0<1003>::deserialize(transfer);
        this->update_tibia_angle(transfer.metadata.remote_node_id, as5048_b_angle.data.value);
      }))
    {
      printf("[ERROR] OpenCyphalAnglePositionSensorBulkReader: failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1003>'");
    }
  }

  void update_femur_angle(CanardNodeID const node_id, float const femur_angle_deg)
  {
    std::lock_guard<std::mutex> lock(_mtx);
  }

  void update_tibia_angle(CanardNodeID const node_id, float const tibia_angle_deg)
  {
    std::lock_guard<std::mutex> lock(_mtx);
  }

  void doBulkRead()
  {
    std::lock_guard<std::mutex> lock(_mtx);
  }

private:
  std::mutex _mtx;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
