/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_
#define GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>
#include <tuple>

#include <rclcpp/rclcpp.hpp>

#include <l3xz_io/types/LegJointKey.h>
#include <l3xz_io/types/HeadJointKey.h>

#include <l3xz_io/control/dynamixel/MX28.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionReader
{
public:
  DynamixelAnglePositionReader() = delete;

  static std::tuple<std::map<LegJointKey, float>, std::map<HeadJointKey, float>> doBulkRead(SharedMX28 mx28_ctrl, rclcpp::Logger const logger);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
