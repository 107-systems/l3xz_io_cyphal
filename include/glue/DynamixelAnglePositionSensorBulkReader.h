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

#include <driver/dynamixel/MX28.h>

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

class DynamixelAnglePositionSensorBulkReader
{
public:
  DynamixelAnglePositionSensorBulkReader() = delete;

  static std::map<DynamixelServoName, float> doBulkRead(dynamixel::SharedMX28 mx28_ctrl)
  {
    dynamixel::MX28::AngleDataSet const angle_data_set = mx28_ctrl->getAngle(DYNAMIXEL_ID_LIST);

    std::map<DynamixelServoName, float> dynamixel_angle_position_map;

    for (auto [id, angle_deg] : angle_data_set)
    {
      printf("[DEBUG] id %d = %.2f", id, angle_deg);
      float const corrected_angle_deg = (angle_deg - 180.0f);

      DynamixelServoName const key = toServoName(id);
      dynamixel_angle_position_map[key] = corrected_angle_deg;
    }

    return dynamixel_angle_position_map;
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
