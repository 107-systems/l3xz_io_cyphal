/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef KINEMATIC_IK_INPUT_H_
#define KINEMATIC_IK_INPUT_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cmath>

#include <string>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::kinematic
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class IK_Input
{
public:
  IK_Input(double const tibia_tip_x,
           double const tibia_tip_y,
           double const tibia_tip_z,
           double const coxa_angle_deg,
           double const femur_angle_deg,
           double const tibia_angle_deg)
  : _frame{KDL::Vector(tibia_tip_x, tibia_tip_y, tibia_tip_z)}
  {
    _joint_positions = KDL::JntArray(3);
    _joint_positions(0) = coxa_angle_deg  * M_PI / 180.0;
    _joint_positions(1) = femur_angle_deg * M_PI / 180.0;
    _joint_positions(2) = tibia_angle_deg * M_PI / 180.0;
  }

  inline KDL::Frame    tibia_tip_frame() const { return _frame; }
  inline KDL::JntArray joint_positions() const { return _joint_positions; }


private:
  KDL::Frame const _frame;
  KDL::JntArray _joint_positions;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */

#endif /* KINEMATIC_IK_INPUT_H_ */
