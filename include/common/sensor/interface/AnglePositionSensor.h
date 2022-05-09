/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef COMMON_SENSOR_INTERFACE_ANGLE_POSITION_SENSOR_H_
#define COMMON_SENSOR_INTERFACE_ANGLE_POSITION_SENSOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Base.hpp"

#include <memory>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::sensor::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class AnglePositionSensor : public Base<float>
{
public:
           AnglePositionSensor(std::string const & name) : Base(name), _val{std::nullopt} { }
  virtual ~AnglePositionSensor() { }

  virtual std::optional<float> get() const override { return _val; }

protected:
  void set(float const val) { _val = val;}

private:
  std::optional<float> _val;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<AnglePositionSensor> SharedAnglePositionSensor;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::sensor::interface */

#endif /* COMMON_SENSOR_INTERFACE_ANGLE_POSITION_SENSOR_H_ */
