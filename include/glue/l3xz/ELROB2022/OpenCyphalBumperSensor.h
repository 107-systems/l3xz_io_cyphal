/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_H_
#define GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/sensor/interface/BumperSensor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class OpenCyphalBumperSensor : public common::sensor::interface::BumperSensor
{
public:
  OpenCyphalBumperSensor(std::string const & name)
  : BumperSensor(name)
  , _is_pressed{std::nullopt}
  { }

  virtual std::optional<bool> get() const override { return _is_pressed; }

  void update(bool const is_pressed) { _is_pressed = is_pressed; }

private:
  std::optional<bool> _is_pressed;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<OpenCyphalBumperSensor> SharedOpenCyphalBumperSensor;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_BUMPER_SENSOR_H_ */
