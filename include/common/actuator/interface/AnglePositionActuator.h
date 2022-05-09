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

#include "Base.h"

#include <memory>
#include <optional>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::actuator::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class AnglePositionActuator : public Base

{
public:
           AnglePositionActuator(std::string const & name);
  virtual ~AnglePositionActuator() { }

  void set(float const val);
  virtual std::string toStr() const override;

protected:
  std::optional<float> get() const;

private:
  std::optional<float> _val;
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
