/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_H_
#define COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>
#include <optional>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class AnglePositionActuator
{
public:
  AnglePositionActuator(std::string const & name);

  void set(float const val);
  std::string toStr() const;

protected:
  std::optional<float> get() const;

private:
  std::string const _name;
  std::optional<float> _val;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::actuator::interface */

#endif /* COMMON_SENSOR_INTERFACE_ANGLE_POSITION_ACTUATOR_H_ */
