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

#include <glue/DynamixelServoName.h>
#include <glue/l3xz/ELROB2022/Const.h>

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
    std::map<dynamixel::Dynamixel::Id, DynamixelServoName> const DYNAMIXEL_ID_TO_SERVO_KEY =
    {
      {1, DynamixelServoName::LeftFront_Coxa},
      {2, DynamixelServoName::LeftMiddle_Coxa},
      {3, DynamixelServoName::LeftBack_Coxa},
      {4, DynamixelServoName::RightBack_Coxa},
      {5, DynamixelServoName::RightMiddle_Coxa},
      {6, DynamixelServoName::RightFront_Coxa},
      {7, DynamixelServoName::Head_Pan},
      {8, DynamixelServoName::Head_Tilt},
    };

    dynamixel::MX28::AngleDataSet const angle_data_set = mx28_ctrl->getAngle(l3xz::ELROB2022::DYNAMIXEL_ID_VECT);

    std::map<DynamixelServoName, float> dynamixel_angle_position_map;

    for (auto [id, angle_deg] : angle_data_set)
    {
      printf("[DEBUG] id %d = %.2f", id, angle_deg);
      float const corrected_angle_deg = (angle_deg - 180.0f);

      DynamixelServoName const key = DYNAMIXEL_ID_TO_SERVO_KEY.at(id);
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
