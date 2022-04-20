/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef COMMON_INTERFACE_SENSOR_H_
#define COMMON_INTERFACE_SENSOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>
#include <optional>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::common::interface::sensor
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
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::common::interface::sensor */

#endif /* COMMON_INTERFACE_SENSOR_H_ */
