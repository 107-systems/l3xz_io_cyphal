/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <glue/DynamixelAnglePositionWriter.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

DynamixelAnglePositionWriter::DynamixelAnglePositionWriter()
{ }

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void DynamixelAnglePositionWriter::update(DynamixelServoName const name, float const angle_deg)
{
  _dynamixel_angle_map[toServoId(name)] = angle_deg;
}

bool DynamixelAnglePositionWriter::doBulkWrite(dynamixel::SharedMX28 mx28_ctrl)
{
  dynamixel::MX28::AngleDataSet angle_data_set;

  for (auto [id, angle_deg] : _dynamixel_angle_map)
  {
    float const corrected_angle_deg = (angle_deg + 180.0f);
    angle_data_set[id] = corrected_angle_deg;
  }

  return mx28_ctrl->setAngle(angle_data_set);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
