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

class ControllerOutput
{
public:
  ControllerOutput(double const pan_angle_target, double const tilt_angle_target)
  : _pan_angle_target {pan_angle_target}
  , _tilt_angle_target{tilt_angle_target}
  { }

  ControllerOutput(ControllerOutput const & other)
  : _pan_angle_target {other[Angle::Pan]}
  , _tilt_angle_target{other[Angle::Tilt]}
  { }

  enum class Angle {Pan, Tilt};

  inline double const operator[](Angle const angle) const
  {
    switch(angle)
    {
    case Angle::Pan : return _pan_angle_target; break;
    case Angle::Tilt: return _tilt_angle_target; break;
    default: throw std::runtime_error("ControllerOutput::operator[] error, invalid parameter for 'angle'"); break;
    }
  }

private:
  double _pan_angle_target,
         _tilt_angle_target;
};

/**************************************************************************************
 * FREE FUNCTION DECLARATION
 **************************************************************************************/

bool operator == (ControllerOutput const & lhs, ControllerOutput const & rhs);
bool operator != (ControllerOutput const & lhs, ControllerOutput const & rhs);

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head */

#endif /* HEAD_CONTROLLER_OUTPUT_H_ */
