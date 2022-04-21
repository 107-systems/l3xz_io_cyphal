/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/common/actuator/interface/AnglePositionActuator.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionActuator : public common::actuator::interface::AnglePositionActuator
{
public:
  DynamixelAnglePositionActuator(std::string const & name, float const initial_value)
  : AnglePositionActuator(name)
  {
    set(initial_value);
  }

  float getAngleDeg() const {
    return get().value();
  }
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<DynamixelAnglePositionActuator> SharedDynamixelAnglePositionActuator;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_H_ */
