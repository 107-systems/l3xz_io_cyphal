/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_BULK_WRITER_H_
#define GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_BULK_WRITER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/driver/dynamixel/MX28.h>

#include "DynamixelAnglePositionActuator.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionActuatorBulkWriter
{
public:
  DynamixelAnglePositionActuatorBulkWriter(driver::SharedMX28 mx28_ctrl,
                                           SharedDynamixelAnglePositionActuator coxa_leg_front_left,
                                           SharedDynamixelAnglePositionActuator coxa_leg_front_right,
                                           SharedDynamixelAnglePositionActuator coxa_leg_middle_left,
                                           SharedDynamixelAnglePositionActuator coxa_leg_middle_right,
                                           SharedDynamixelAnglePositionActuator coxa_leg_back_left,
                                           SharedDynamixelAnglePositionActuator coxa_leg_back_right,
                                           SharedDynamixelAnglePositionActuator sensor_head_pan,
                                           SharedDynamixelAnglePositionActuator sensor_head_tilt)
  : _mx28_ctrl{mx28_ctrl}
  , DYNAMIXEL_ID_TO_ANGLE_POSITION_ACTUATOR
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

  bool doBulkWrite()
  {
    driver::MX28::AngleDataSet angle_data_set;

    for (auto [id, angle_pos_actuator] : DYNAMIXEL_ID_TO_ANGLE_POSITION_ACTUATOR)
      angle_data_set[id] = angle_pos_actuator->getAngleDeg();

    return _mx28_ctrl->setAngle(angle_data_set);
  }

private:
  driver::SharedMX28 _mx28_ctrl;
  std::map<driver::Dynamixel::Id, SharedDynamixelAnglePositionActuator> const DYNAMIXEL_ID_TO_ANGLE_POSITION_ACTUATOR;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_BULK_WRITER_H_ */
