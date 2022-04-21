/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef COMMON_SENSOR_INTERFACE_ANGLE_POSITION_SENSOR_H_
#define COMMON_SENSOR_INTERFACE_ANGLE_POSITION_SENSOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>
#include <memory>
#include <optional>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::sensor::interface
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class AnglePositionSensor
{
public:

   AnglePositionSensor(std::string const & name);
  ~AnglePositionSensor();


  std::optional<float> get() const;

  void update(float const val);

  std::string toStr() const;


private:
  std::string const _name;
  float _val;
  bool _is_valid;
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
