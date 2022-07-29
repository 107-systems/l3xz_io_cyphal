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

#include <driver/dynamixel/MX28.h>

#include <types/LegJointKey.h>
#include <types/HeadJointKey.h>

#include <glue/DynamixelIdList.h>
#include <glue/DynamixelServoName.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionReader
{
public:
  DynamixelAnglePositionReader() = delete;

  static std::tuple<std::map<LegJointKey, float>, std::map<HeadJointKey, float>> doBulkRead(dynamixel::SharedMX28 mx28_ctrl);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
