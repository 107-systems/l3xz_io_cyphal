/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_H_
#define GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/sensor/interface/AnglePositionSensor.h>

#include <Const.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class OpenCyphalAnglePositionSensor : public common::sensor::interface::AnglePositionSensor
{
public:
  OpenCyphalAnglePositionSensor(std::string const & name, Leg const leg, Joint const joint)
  : AnglePositionSensor(name)
  , LEG(leg)
  , JOINT(joint)
  , _angle_deg{std::nullopt}
  { }

  virtual std::optional<float> get() const override { return _angle_deg; }

  void update(float const angle_deg) { _angle_deg = angle_deg; }

  Leg const LEG;
  Joint const JOINT;

private:
  std::optional<float> _angle_deg;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<OpenCyphalAnglePositionSensor> SharedOpenCyphalAnglePositionSensor;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_ANGLE_POSITION_SENSOR_H_ */
