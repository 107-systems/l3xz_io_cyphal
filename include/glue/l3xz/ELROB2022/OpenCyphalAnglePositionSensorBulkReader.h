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

#include <map>
#include <mutex>
#include <vector>

#include <canard.h>

#include <glue/OpenCyphalNodeIdList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class OpenCyphalAnglePositionSensorBulkReader
{
public:
  OpenCyphalAnglePositionSensorBulkReader()
  { }

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

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
