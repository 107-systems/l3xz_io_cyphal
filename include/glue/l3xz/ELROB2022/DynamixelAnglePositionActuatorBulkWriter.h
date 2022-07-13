/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_BULK_WRITER_H_
#define GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_BULK_WRITER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <driver/dynamixel/MX28.h>

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
  DynamixelAnglePositionActuatorBulkWriter(driver::SharedMX28 mx28_ctrl)
  : _mx28_ctrl{mx28_ctrl}
  { }

  void update(driver::Dynamixel::Id const id, float const angle_deg)
  {
    _dynamixel_angle_map[id] = angle_deg;
  }

  bool doBulkWrite()
  {
    driver::MX28::AngleDataSet angle_data_set;

    for (auto [id, angle_deg] : _dynamixel_angle_map)
    {
      float const corrected_angle_deg = (angle_deg + 180.0f);
      angle_data_set[id] = corrected_angle_deg;
    }

    return _mx28_ctrl->setAngle(angle_data_set);
  }

private:
  driver::SharedMX28 _mx28_ctrl;
  std::map<driver::Dynamixel::Id, float> _dynamixel_angle_map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_BULK_WRITER_H_ */
