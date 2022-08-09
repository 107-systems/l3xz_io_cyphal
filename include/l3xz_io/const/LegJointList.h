/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef CONST_LEG_JOINT_LIST_H_
#define CONST_LEG_JOINT_LIST_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <list>

#include <l3xz_io/types/LegJointKey.h>

/**************************************************************************************
 * CONST
 **************************************************************************************/

static std::list<LegJointKey> const LEG_JOINT_LIST =
{
  make_key(Leg::LeftFront,   Joint::Coxa),
  make_key(Leg::LeftFront,   Joint::Femur),
  make_key(Leg::LeftFront,   Joint::Tibia),
  make_key(Leg::LeftMiddle,  Joint::Coxa),
  make_key(Leg::LeftMiddle,  Joint::Femur),
  make_key(Leg::LeftMiddle,  Joint::Tibia),
  make_key(Leg::LeftBack,    Joint::Coxa),
  make_key(Leg::LeftBack,    Joint::Femur),
  make_key(Leg::LeftBack,    Joint::Tibia),
  make_key(Leg::RightBack,   Joint::Coxa),
  make_key(Leg::RightBack,   Joint::Femur),
  make_key(Leg::RightBack,   Joint::Tibia),
  make_key(Leg::RightMiddle, Joint::Coxa),
  make_key(Leg::RightMiddle, Joint::Femur),
  make_key(Leg::RightMiddle, Joint::Tibia),
  make_key(Leg::RightFront,  Joint::Coxa),
  make_key(Leg::RightFront,  Joint::Femur),
  make_key(Leg::RightFront,  Joint::Tibia),
};

#endif /* CONST_LEG_JOINT_LIST_H_ */
