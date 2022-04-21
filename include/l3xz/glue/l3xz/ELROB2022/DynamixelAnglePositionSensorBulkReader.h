/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_
#define GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_BULK_READER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/driver/dynamixel/MX28.h>

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
                                         SharedDynamixelAnglePositionSensor coxa_leg_front_left,
                                         SharedDynamixelAnglePositionSensor coxa_leg_front_right,
                                         SharedDynamixelAnglePositionSensor coxa_leg_middle_left,
                                         SharedDynamixelAnglePositionSensor coxa_leg_middle_right,
                                         SharedDynamixelAnglePositionSensor coxa_leg_back_left,
                                         SharedDynamixelAnglePositionSensor coxa_leg_back_right,
                                         SharedDynamixelAnglePositionSensor sensor_head_pan,
                                         SharedDynamixelAnglePositionSensor sensor_head_tilt)
  : _mx28_ctrl{mx28_ctrl}
  , DYNAMIXEL_ID_TO_ANGLE_POSITION_SENSOR
  {
    {1, coxa_leg_front_left},
    {2, coxa_leg_front_right},
    {3, coxa_leg_middle_left},
    {4, coxa_leg_middle_right},
    {5, coxa_leg_back_left},
    {6, coxa_leg_back_right},
    {7, sensor_head_pan},
    {8, sensor_head_tilt},
  }
  { }

  void doBulkRead()
  {
    for (auto [id, angle_deg] : _mx28_ctrl->getAngle(DYNAMIXEL_ID_VECT))
      DYNAMIXEL_ID_TO_ANGLE_POSITION_SENSOR.at(id)->notify(angle_deg);
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
