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

#include "Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionSensorBulkReader
{
public:
  DynamixelAnglePositionSensorBulkReader() = delete;

  enum class ServoKey
  {
    LeftFront_Coxa,
    LeftMiddle_Coxa,
    LeftBack_Coxa,
    RightBack_Coxa,
    RightMiddle_Coxa,
    RightFront_Coxa,
    Head_Pan,
    Head_Tilt,
  };

  static std::map<ServoKey, float> doBulkRead(driver::SharedMX28 mx28_ctrl)
  {
    std::map<driver::Dynamixel::Id, ServoKey> const DYNAMIXEL_ID_TO_SERVO_KEY =
    {
      {1, ServoKey::LeftFront_Coxa},
      {2, ServoKey::LeftMiddle_Coxa},
      {3, ServoKey::LeftBack_Coxa},
      {4, ServoKey::RightBack_Coxa},
      {5, ServoKey::RightMiddle_Coxa},
      {6, ServoKey::RightFront_Coxa},
      {7, ServoKey::Head_Pan},
      {8, ServoKey::Head_Tilt},
    };

    driver::MX28::AngleDataSet const angle_data_set = mx28_ctrl->getAngle(DYNAMIXEL_ID_VECT);

    std::map<ServoKey, float> dynamixel_angle_position_map;

    for (auto [id, angle_deg] : angle_data_set)
    {
      printf("[DEBUG] id %d = %.2f", id, angle_deg);
      float const corrected_angle_deg = (angle_deg - 180.0f);

      ServoKey const key = DYNAMIXEL_ID_TO_SERVO_KEY.at(id);
      dynamixel_angle_position_map[key] = corrected_angle_deg;
    }

    return dynamixel_angle_position_map;
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
