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

#include <rclcpp/rclcpp.hpp>

#include <types/LegJointKey.h>

#include <phy/opencyphal/Node.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class HydraulicAnglePositionReader
{
public:
  HydraulicAnglePositionReader(phy::opencyphal::Node & node,
                               rclcpp::Logger const logger);

  std::map<LegJointKey, float> doBulkRead();

private:
  std::mutex _mtx;
  std::map<LegJointKey, float> _leg_angle_position_map;

  static LegJointKey femur_toLegJointKey(CanardNodeID const node_id);
  static LegJointKey tibia_toLegJointKey(CanardNodeID const node_id);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
