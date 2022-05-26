/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_MAP_H_
#define COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_MAP_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>
#include <tuple>

#include "AnglePositionActuator.h"

#include <Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

typedef std::map<LegJointKey, SharedAnglePositionActuator> AnglePositionActuatorMap;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */

#endif /* COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_MAP_H_ */
