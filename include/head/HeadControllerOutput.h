/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_OUTPUT_H_
#define HEAD_CONTROLLER_OUTPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/AnglePositionActuator.h>

#include <stdexcept>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class HeadControllerOutput
{
public:
  HeadControllerOutput()
  : _pan_angle_target {0.0}
  , _tilt_angle_target{0.0}
  { }

  enum class Angle {Pan, Tilt };

  inline double & operator[](Angle const angle) {
    switch(angle)
    {
    case Angle::Pan : return _pan_angle_target; break;
    case Angle::Tilt: return _tilt_angle_target; break;
    default: throw std::runtime_error("HeadControllerOutput::operator[] error, invalid parameter for 'angle'"); break;
    }
  }

  inline double const operator[](Angle const angle) const {
    switch(angle)
    {
    case Angle::Pan : return _pan_angle_target; break;
    case Angle::Tilt: return _tilt_angle_target; break;
    default: throw std::runtime_error("HeadControllerOutput::operator[] error, invalid parameter for 'angle'"); break;
    }
  }

private:
  double _pan_angle_target,
         _tilt_angle_target;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head */

#endif /* HEAD_CONTROLLER_OUTPUT_H_ */
