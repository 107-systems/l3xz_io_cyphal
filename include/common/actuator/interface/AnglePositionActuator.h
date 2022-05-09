/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_H_
#define COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Base.hpp"

#include <memory>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class AnglePositionActuator : public Base<float>

{
public:
           AnglePositionActuator(std::string const & name) : Base(std::string("[Angle Position Actuator] \"") + name + std::string("\"")) { }
  virtual ~AnglePositionActuator() { }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

typedef std::shared_ptr<AnglePositionActuator> SharedAnglePositionActuator;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */

#endif /* COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_H_ */
