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

#include <glue/DynamixelServoName.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionWriter
{
public:
  DynamixelAnglePositionWriter();

  void update(DynamixelServoName const name, float const angle_deg);
  bool doBulkWrite(dynamixel::SharedMX28 mx28_ctrl);


private:
  std::map<dynamixel::Dynamixel::Id, float> _dynamixel_angle_map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_BULK_WRITER_H_ */
