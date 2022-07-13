/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef KINEMATIC_FK_INPUT_H_
#define KINEMATIC_FK_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <cmath>

#include <kdl/jntarray.hpp>

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
  {
    _joint_positions = KDL::JntArray(3);
    _joint_positions(0) = coxa_angle_deg  * M_PI / 180.0;
    _joint_positions(1) = femur_angle_deg * M_PI / 180.0;
    _joint_positions(2) = tibia_angle_deg * M_PI / 180.0;
  }

  inline KDL::JntArray joint_positions() const { return _joint_positions; }

private:
  KDL::JntArray _joint_positions;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */

#endif /* KINEMATIC_FK_INPUT_H_ */
