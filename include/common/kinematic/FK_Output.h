/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef KINEMATIC_FK_OUTPUT_H_
#define KINEMATIC_FK_OUTPUT_H_

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::kinematic
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class FK_Output
{
public:
  FK_Output(double const tibia_tip_x, double const tibia_tip_y, double const tibia_tip_z)
  : _tibia_tip_x{tibia_tip_x}
  , _tibia_tip_y{tibia_tip_y}
  , _tibia_tip_z{tibia_tip_z}
  { }

  inline double tibia_tip_x() const { return _tibia_tip_x; }
  inline double tibia_tip_y() const { return _tibia_tip_y; }
  inline double tibia_tip_z() const { return _tibia_tip_z; }

private:
  double const _tibia_tip_x;
  double const _tibia_tip_y;
  double const _tibia_tip_z;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */

#endif /* KINEMATIC_FK_OUTPUT_H_ */
