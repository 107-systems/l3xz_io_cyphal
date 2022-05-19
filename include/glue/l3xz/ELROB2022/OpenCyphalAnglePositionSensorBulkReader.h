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

#include <map>
#include <mutex>

#include <canard.h>

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
  OpenCyphalAnglePositionSensorBulkReader(SharedOpenCyphalAnglePositionSensor angle_sensor_femur_leg_front_left,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_tibia_leg_front_left,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_femur_leg_middle_left,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_tibia_leg_middle_left,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_femur_leg_back_left,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_tibia_leg_back_left,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_femur_leg_front_right,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_tibia_leg_front_right,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_femur_leg_middle_right,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_tibia_leg_middle_right,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_femur_leg_back_right,
                                          SharedOpenCyphalAnglePositionSensor angle_sensor_tibia_leg_back_right)
  : _angle_sensor_femur_leg_front_left  {angle_sensor_femur_leg_front_left}
  , _angle_sensor_tibia_leg_front_left  {angle_sensor_tibia_leg_front_left}
  , _angle_sensor_femur_leg_middle_left {angle_sensor_femur_leg_middle_left}
  , _angle_sensor_tibia_leg_middle_left {angle_sensor_tibia_leg_middle_left}
  , _angle_sensor_femur_leg_back_left   {angle_sensor_femur_leg_back_left}
  , _angle_sensor_tibia_leg_back_left   {angle_sensor_tibia_leg_back_left}
  , _angle_sensor_femur_leg_front_right {angle_sensor_femur_leg_front_right}
  , _angle_sensor_tibia_leg_front_right {angle_sensor_tibia_leg_front_right}
  , _angle_sensor_femur_leg_middle_right{angle_sensor_femur_leg_middle_right}
  , _angle_sensor_tibia_leg_middle_right{angle_sensor_tibia_leg_middle_right}
  , _angle_sensor_femur_leg_back_right  {angle_sensor_femur_leg_back_right}
  , _angle_sensor_tibia_leg_back_right  {angle_sensor_tibia_leg_back_right}
  {

  }

  void update_femur_angle(CanardNodeID const node_id, float const femur_angle_deg)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    _femur_angle_map[node_id] = femur_angle_deg;
  }
  void update_tibia_angle(CanardNodeID const node_id, float const tibia_angle_deg)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    _tibia_angle_map[node_id] = tibia_angle_deg;
  }

  /* It's not really a bulk read but a bulk copy
   * of all received data so far.
   */
  void doBulkRead()
  {
    std::lock_guard<std::mutex> lock(_mtx);

    if (_femur_angle_map.count(255) > 0) _angle_sensor_femur_leg_front_left->update(_femur_angle_map.at(255));
    if (_tibia_angle_map.count(255) > 0) _angle_sensor_tibia_leg_front_left->update(_tibia_angle_map.at(255));
  }

private:
  std::mutex _mtx;

  std::map<CanardNodeID, float> _femur_angle_map;
  std::map<CanardNodeID, float> _tibia_angle_map;

  SharedOpenCyphalAnglePositionSensor _angle_sensor_femur_leg_front_left,
                                      _angle_sensor_tibia_leg_front_left,
                                      _angle_sensor_femur_leg_middle_left,
                                      _angle_sensor_tibia_leg_middle_left,
                                      _angle_sensor_femur_leg_back_left,
                                      _angle_sensor_tibia_leg_back_left,
                                      _angle_sensor_femur_leg_front_right,
                                      _angle_sensor_tibia_leg_front_right,
                                      _angle_sensor_femur_leg_middle_right,
                                      _angle_sensor_tibia_leg_middle_right,
                                      _angle_sensor_femur_leg_back_right,
                                      _angle_sensor_tibia_leg_back_right;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
