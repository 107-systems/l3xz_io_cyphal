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

#include <cassert>

#include "Const.h"
#include "DynamixelAnglePositionSensor.h"

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
  DynamixelAnglePositionSensorBulkReader(driver::SharedMX28 mx28_ctrl,
                                         SharedDynamixelAnglePositionSensor angle_sensor_left_front_coxa,
                                         SharedDynamixelAnglePositionSensor angle_sensor_left_middle_coxa,
                                         SharedDynamixelAnglePositionSensor angle_sensor_left_back_coxa,
                                         SharedDynamixelAnglePositionSensor angle_sensor_right_back_coxa,
                                         SharedDynamixelAnglePositionSensor angle_sensor_right_middle_coxa,
                                         SharedDynamixelAnglePositionSensor angle_sensor_right_front_coxa,
                                         SharedDynamixelAnglePositionSensor angle_sensor_sensor_head_pan,
                                         SharedDynamixelAnglePositionSensor angle_sensor_sensor_head_tilt)
  : _mx28_ctrl{mx28_ctrl}
  , DYNAMIXEL_ID_TO_ANGLE_POSITION_SENSOR
  {
    {1, angle_sensor_left_front_coxa},
    {2, angle_sensor_left_middle_coxa},
    {3, angle_sensor_left_back_coxa},
    {4, angle_sensor_right_back_coxa},
    {5, angle_sensor_right_middle_coxa},
    {6, angle_sensor_right_front_coxa},
    {7, angle_sensor_sensor_head_pan},
    {8, angle_sensor_sensor_head_tilt},
  }
  { }

  void doBulkRead()
  {
    driver::MX28::AngleDataSet const angle_data_set = _mx28_ctrl->getAngle(DYNAMIXEL_ID_VECT);

    for (auto [id, angle_deg] : angle_data_set)
    {
      printf("[DEBUG] id %d = %.2f", id, angle_deg);
      float const corrected_angle_deg = (angle_deg - 180.0f);
      DYNAMIXEL_ID_TO_ANGLE_POSITION_SENSOR.at(id)->update(corrected_angle_deg);
    }
  }

private:
  driver::SharedMX28 _mx28_ctrl;
  std::map<driver::Dynamixel::Id, SharedDynamixelAnglePositionSensor> const DYNAMIXEL_ID_TO_ANGLE_POSITION_SENSOR;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_ */
