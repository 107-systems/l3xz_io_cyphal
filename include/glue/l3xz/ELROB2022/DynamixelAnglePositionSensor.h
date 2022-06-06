/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_H_
#define GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Const.h>
#include <common/sensor/interface/AnglePositionSensor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionSensor : public common::sensor::interface::AnglePositionSensor
{
public:
  DynamixelAnglePositionSensor(std::string const & name)
  : AnglePositionSensor(name)
  , _angle_deg{std::nullopt} { }

  virtual std::optional<float> get() const override { return _angle_deg; }

  void update(float const angle_deg) { _angle_deg = angle_deg; }

private:
  std::optional<float> _angle_deg;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<DynamixelAnglePositionSensor> SharedDynamixelAnglePositionSensor;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_SENSOR_H_ */
