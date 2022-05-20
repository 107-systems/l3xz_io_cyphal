/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef KINEMATIC_FK_INPUT_H_
#define KINEMATIC_FK_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <cmath>

#include <Const.h>

#include <stdexcept>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::kinematic
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class FK_Input
{
public:
  FK_Input(double const coxa_angle_deg, double const femur_angle_deg, double const tibia_angle_deg)
  : _coxa_angle_rad {coxa_angle_deg  * M_PI / 180.0}
  , _femur_angle_rad{femur_angle_deg * M_PI / 180.0}
  , _tibia_angle_rad{tibia_angle_deg * M_PI / 180.0}
  { }

  double angle_rad(Joint const joint) const
  {
    switch(joint)
    {
    case Joint::Coxa : return _coxa_angle_rad;  break;
    case Joint::Femur: return _femur_angle_rad; break;
    case Joint::Tibia: return _tibia_angle_rad; break;
    case Joint::Invalid:
    default: throw std::runtime_error("FK_Input::angle() invalid param"); break;
    }
  }

private:
  double const _coxa_angle_rad;
  double const _femur_angle_rad;
  double const _tibia_angle_rad;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */

#endif /* KINEMATIC_FK_INPUT_H_ */
