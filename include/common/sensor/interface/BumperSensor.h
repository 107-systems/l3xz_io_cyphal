/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef COMMON_SENSOR_INTERFACE_BUMPER_SENSOR_H_
#define COMMON_SENSOR_INTERFACE_BUMPER_SENSOR_H_

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

class BumperSensor : public Base<bool>
{
public:
           BumperSensor(std::string const & name) : Base(std::string("[Bumper Sensor] \"") + name + std::string("\"")) { }
  virtual ~BumperSensor() { }

  virtual std::string toStr() const override;

  std::optional<bool> isPressed() const;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<BumperSensor> SharedBumperSensor;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::sensor::interface */

#endif /* COMMON_SENSOR_INTERFACE_BUMPER_SENSOR_H_ */
