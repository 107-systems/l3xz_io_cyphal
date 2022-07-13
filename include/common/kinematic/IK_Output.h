/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef KINEMATIC_IK_OUTPUT_H_
#define KINEMATIC_IK_OUTPUT_H_

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

class IK_Output
{
public:
  IK_Output(double const coxa_angle_rad, double const femur_angle_rad, double const tibia_angle_rad);

  double angle_deg(Joint const joint) const
  {
    switch(joint)
    {
    case Joint::Coxa : return _coxa_angle_deg;  break;
    case Joint::Femur: return _femur_angle_deg; break;
    case Joint::Tibia: return _tibia_angle_deg; break;
    default: throw std::runtime_error("IK_Output::angle_deg() invalid param"); break;
    }
  }

  std::string toStr() const;


private:
  double const _coxa_angle_deg;
  double const _femur_angle_deg;
  double const _tibia_angle_deg;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */

#endif /* KINEMATIC_IK_OUTPUT_H_ */
