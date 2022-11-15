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

#include <l3xz_io/types/LegJointKey.h>
#include <l3xz_io/types/HeadJointKey.h>

#include <l3xz_io/control/dynamixel/DynamixelMX28.h>
#include <l3xz_io/control/dynamixel/DynamixelServoName.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionWriter
{
public:
  DynamixelAnglePositionWriter();

  void update(LegJointKey const joint, float const angle_deg);
  void update(HeadJointKey const joint, float const angle_deg);
  bool doBulkWrite(SharedMX28 mx28_ctrl);


private:
  std::map<dynamixelplusplus::Dynamixel::Id, float> _dynamixel_angle_map;

  void update(DynamixelServoName const name, float const angle_deg);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_BULK_WRITER_H_ */
