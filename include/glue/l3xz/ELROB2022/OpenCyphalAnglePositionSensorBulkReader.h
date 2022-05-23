/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_
#define GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "OpenCyphalAnglePositionSensor.h"

#include <cassert>

#include <map>
#include <mutex>
#include <vector>

#include <canard.h>

#include <ros/ros.h>
#include <ros/console.h>

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
  OpenCyphalAnglePositionSensorBulkReader(SharedOpenCyphalAnglePositionSensor angle_sensor_left_front_femur,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_left_front_tibia,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_left_middle_femur,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_left_middle_tibia,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_left_back_femur,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_left_back_tibia,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_right_back_femur,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_right_back_tibia,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_right_middle_femur,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_right_middle_tibia,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_right_front_femur,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_right_front_tibia)
  : FEMUR_ANGLE_OFFSET_MAP
  {
    {1, 286.08},
    {2,  50.34},
    {3, 324.40},
    {4, 105.85},
    {5, 347.97},
    {6,  60.10}
  }
  , TIBIA_ANGLE_OFFSET_MAP
  {
    {1, 246.04},
    {2, 159.08+25},
    {3, 304.90},
    {4, 234.05+40+8},
    {5,  87.97},
    {6, 124.10}
  }
  , NODE_ID_TO_FEMUR_ANGLE_POSITION_SENSOR_MAP
  {
    {1, angle_sensor_left_front_femur},
    {2, angle_sensor_left_middle_femur},
    {3, angle_sensor_left_back_femur},
    {4, angle_sensor_right_back_femur},
    {5, angle_sensor_right_middle_femur},
    {6, angle_sensor_right_front_femur},
  }
  , NODE_ID_TO_TIBIA_ANGLE_POSITION_SENSOR_MAP
  {
    {1, angle_sensor_left_front_tibia},
    {2, angle_sensor_left_middle_tibia},
    {3, angle_sensor_left_back_tibia},
    {4, angle_sensor_right_back_tibia},
    {5, angle_sensor_right_middle_tibia},
    {6, angle_sensor_right_front_tibia},
  },
  ALLOWED_NODE_ID_VECT{1,2,3,4,5,6}
  { }

  void update_femur_angle(CanardNodeID const node_id, float const femur_angle_deg)
  {
    if (!isValidNodeId(node_id)) {
      ROS_WARN("OpenCyphalAnglePositionSensorBulkReader::update_femur_angle: invalid node id received: %d", node_id);
      return;
    }
    std::lock_guard<std::mutex> lock(_mtx);
    float const corrected_femur_angle_deg = (femur_angle_deg - FEMUR_ANGLE_OFFSET_MAP.at(node_id));
    _femur_angle_map[node_id] = corrected_femur_angle_deg;
  }

  void update_tibia_angle(CanardNodeID const node_id, float const tibia_angle_deg)
  {
    if (!isValidNodeId(node_id)) {
      ROS_WARN("OpenCyphalAnglePositionSensorBulkReader::update_tibia_angle: invalid node id received: %d", node_id);
      return;
    }
    std::lock_guard<std::mutex> lock(_mtx);
    float const corrected_tibia_angle_deg = (tibia_angle_deg - TIBIA_ANGLE_OFFSET_MAP.at(node_id));
    _tibia_angle_map[node_id] = corrected_tibia_angle_deg;
  }

  /* It's not really a bulk read but a bulk copy
   * of all received data so far.
   */
  void doBulkRead()
  {
    std::lock_guard<std::mutex> lock(_mtx);

    for (auto [id, sensor] : NODE_ID_TO_FEMUR_ANGLE_POSITION_SENSOR_MAP) {
      if (_femur_angle_map.count(id) > 0) {
        NODE_ID_TO_FEMUR_ANGLE_POSITION_SENSOR_MAP.at(id)->update(_femur_angle_map.at(id));
      }
    }

    for (auto [id, sensor] : NODE_ID_TO_TIBIA_ANGLE_POSITION_SENSOR_MAP) {
      if (_tibia_angle_map.count(id) > 0) {
        NODE_ID_TO_TIBIA_ANGLE_POSITION_SENSOR_MAP.at(id)->update(_tibia_angle_map.at(id));
      }
    }
  }

private:
  std::mutex _mtx;

  std::map<CanardNodeID, float> _femur_angle_map;
  std::map<CanardNodeID, float> _tibia_angle_map;

  std::map<CanardNodeID, float> const FEMUR_ANGLE_OFFSET_MAP;
  std::map<CanardNodeID, float> const TIBIA_ANGLE_OFFSET_MAP;

  std::map<CanardNodeID, SharedOpenCyphalAnglePositionSensor> const NODE_ID_TO_FEMUR_ANGLE_POSITION_SENSOR_MAP;
  std::map<CanardNodeID, SharedOpenCyphalAnglePositionSensor> const NODE_ID_TO_TIBIA_ANGLE_POSITION_SENSOR_MAP;

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

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
