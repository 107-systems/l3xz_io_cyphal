/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_BULK_READER_H_
#define GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_BULK_READER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>
#include <mutex>

#include <l3xz_io/types/LegJointKey.h>

#include <rclcpp/rclcpp.hpp>

#include <l3xz_io/phy/opencyphal/Node.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class BumperSensorReader
{
public:
  BumperSensorReader(phy::opencyphal::Node & node,
                     rclcpp::Logger const logger);

  std::map<Leg, bool /* is_pressed */> doBulkRead();

private:
  std::mutex _mtx;
  std::map<Leg, bool /* is_pressed */> _tibia_tip_bumper_map;

  static Leg toLeg(CanardNodeID const node_id);

  bool subscribeTibiaTipBumberMessage(phy::opencyphal::Node & node);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_BULK_READER_H_ */
