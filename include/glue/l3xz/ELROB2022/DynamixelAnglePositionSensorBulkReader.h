/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
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
  {
    /* Prevent mis-wirings by checking assumptions. */
    assert(angle_sensor_left_front_coxa->LEG     == Leg::FrontLeft);
    assert(angle_sensor_left_front_coxa->JOINT   == Joint::Coxa);

    assert(angle_sensor_left_middle_coxa->LEG    == Leg::MiddleLeft);
    assert(angle_sensor_left_middle_coxa->JOINT  == Joint::Coxa);

    assert(angle_sensor_left_back_coxa->LEG      == Leg::BackLeft);
    assert(angle_sensor_left_back_coxa->JOINT    == Joint::Coxa);

    assert(angle_sensor_right_back_coxa->LEG     == Leg::BackRight);
    assert(angle_sensor_right_back_coxa->JOINT   == Joint::Coxa);

    assert(angle_sensor_right_middle_coxa->LEG   == Leg::MiddleRight);
    assert(angle_sensor_right_middle_coxa->JOINT == Joint::Coxa);

    assert(angle_sensor_right_front_coxa->LEG    == Leg::FrontRight);
    assert(angle_sensor_right_front_coxa->JOINT  == Joint::Coxa);
  }

  void doBulkRead()
  {
    driver::MX28::AngleDataSet const angle_data_set = _mx28_ctrl->getAngle(DYNAMIXEL_ID_VECT);

    for (auto [id, angle_deg] : angle_data_set) {
      ROS_DEBUG("id %d = %.2f", id, angle_deg);
      DYNAMIXEL_ID_TO_ANGLE_POSITION_SENSOR.at(id)->update(angle_deg);
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
