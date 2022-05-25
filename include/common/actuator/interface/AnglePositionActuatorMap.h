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

typedef std::tuple<Leg, Joint> AnglePositionActuatorKey;
typedef SharedAnglePositionActuator AnglePositionActuatorValue;
typedef std::map<AnglePositionActuatorKey, AnglePositionActuatorValue> AnglePositionActuatorMap;

struct angle_position_actuator_map_key_equal : public std::binary_function<AnglePositionActuatorKey, AnglePositionActuatorKey, bool>
{
  bool operator()(const AnglePositionActuatorKey & v0, const AnglePositionActuatorKey & v1) const
  {
    return (
            std::get<0>(v0) == std::get<0>(v1) &&
            std::get<1>(v0) == std::get<1>(v1)
           );
  }
};

inline AnglePositionActuatorKey angle_position_actuator_map_make_key(Leg const leg, Joint const joint)
{
  return std::tuple(leg, joint);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */

#endif /* COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_MAP_H_ */
