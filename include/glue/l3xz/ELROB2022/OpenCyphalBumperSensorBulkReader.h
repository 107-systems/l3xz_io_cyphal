/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_BULK_READER_H_
#define GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_BULK_READER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "OpenCyphalBumperSensor.h"

#include <cassert>

#include <map>
#include <mutex>
#include <vector>

#include <canard.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class OpenCyphalBumperSensorBulkReader
{
public:
  OpenCyphalBumperSensorBulkReader(SharedOpenCyphalBumperSensor tibia_tip_bumper_left_front,
                                   SharedOpenCyphalBumperSensor tibia_tip_bumper_left_middle,
                                   SharedOpenCyphalBumperSensor tibia_tip_bumper_left_back,
                                   SharedOpenCyphalBumperSensor tibia_tip_bumper_right_back,
                                   SharedOpenCyphalBumperSensor tibia_tip_bumper_right_middle,
                                   SharedOpenCyphalBumperSensor tibia_tip_bumper_right_front)
  : NODE_ID_TO_BUMPER_SENSOR_MAP
  {
    {1, tibia_tip_bumper_left_front},
    {2, tibia_tip_bumper_left_middle},
    {3, tibia_tip_bumper_left_back},
    {4, tibia_tip_bumper_right_back},
    {5, tibia_tip_bumper_right_middle},
    {6, tibia_tip_bumper_right_front},
  }
  , ALLOWED_NODE_ID_VECT{1,2,3,4,5,6}
  { }

  void update_bumper_sensor(CanardNodeID const node_id, bool const is_released)
  {
    if (!isValidNodeId(node_id)) {
      printf("[WARNING] OpenCyphalBumperSensorBulkReader::update_bumper_sensor: invalid node id received: %d", node_id);
      return;
    }
    std::lock_guard<std::mutex> lock(_mtx);
    _bumper_sensor_map[node_id] = !is_released;
  }

  /* It's not really a bulk read but a bulk copy
   * of all received data so far.
   */
  void doBulkRead()
  {
    std::lock_guard<std::mutex> lock(_mtx);

    for (auto [id, sensor] : NODE_ID_TO_BUMPER_SENSOR_MAP)
      if (_bumper_sensor_map.count(id) > 0)
        NODE_ID_TO_BUMPER_SENSOR_MAP.at(id)->update(_bumper_sensor_map.at(id));
  }

private:
  std::mutex _mtx;
  std::map<CanardNodeID, bool> _bumper_sensor_map;
  std::map<CanardNodeID, SharedOpenCyphalBumperSensor> const NODE_ID_TO_BUMPER_SENSOR_MAP;
  std::vector<CanardNodeID> const ALLOWED_NODE_ID_VECT;

  bool isValidNodeId(CanardNodeID const node_id) const
  {
    auto citer = std::find(std::cbegin(ALLOWED_NODE_ID_VECT),
                           std::cend  (ALLOWED_NODE_ID_VECT),
                           node_id);
    bool const node_id_found = (citer != std::cend(ALLOWED_NODE_ID_VECT));
    return node_id_found;
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_BULK_READER_H_ */
