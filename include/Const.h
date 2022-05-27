/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef CONST_H_
#define CONST_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <list>

#include "Types.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

static float constexpr INITIAL_COXA_ANGLE_DEG  = 0.0f;
static float constexpr INITIAL_FEMUR_ANGLE_DEG = 0.0f;
static float constexpr INITIAL_TIBIA_ANGLE_DEG = 0.0f;

static float constexpr INITIAL_PAN_ANGLE_DEG  = 0.0f;
static float constexpr INITIAL_TILT_ANGLE_DEG = 0.0f;

static std::list<Leg> const LEG_LIST =
{
  Leg::LeftFront, Leg::LeftMiddle, Leg::LeftBack, Leg::RightFront, Leg::RightMiddle, Leg::RightBack
};

static std::list<LegJointKey> const LEG_JOINT_LIST =
{
  std::tuple(Leg::LeftFront,   Joint::Coxa),
  std::tuple(Leg::LeftFront,   Joint::Femur),
  std::tuple(Leg::LeftFront,   Joint::Tibia),
  std::tuple(Leg::LeftMiddle,  Joint::Coxa),
  std::tuple(Leg::LeftMiddle,  Joint::Femur),
  std::tuple(Leg::LeftMiddle,  Joint::Tibia),
  std::tuple(Leg::LeftBack,    Joint::Coxa),
  std::tuple(Leg::LeftBack,    Joint::Femur),
  std::tuple(Leg::LeftBack,    Joint::Tibia),
  std::tuple(Leg::RightBack,   Joint::Coxa),
  std::tuple(Leg::RightBack,   Joint::Femur),
  std::tuple(Leg::RightBack,   Joint::Tibia),
  std::tuple(Leg::RightMiddle, Joint::Coxa),
  std::tuple(Leg::RightMiddle, Joint::Femur),
  std::tuple(Leg::RightMiddle, Joint::Tibia),
  std::tuple(Leg::RightFront,  Joint::Coxa),
  std::tuple(Leg::RightFront,  Joint::Femur),
  std::tuple(Leg::RightFront,  Joint::Tibia),
};

static std::list<LegJointKey> const HYDRAULIC_LEG_JOINT_LIST =
{
  std::tuple(Leg::LeftFront,   Joint::Femur),
  std::tuple(Leg::LeftFront,   Joint::Tibia),
  std::tuple(Leg::LeftMiddle,  Joint::Femur),
  std::tuple(Leg::LeftMiddle,  Joint::Tibia),
  std::tuple(Leg::LeftBack,    Joint::Femur),
  std::tuple(Leg::LeftBack,    Joint::Tibia),
  std::tuple(Leg::RightBack,   Joint::Femur),
  std::tuple(Leg::RightBack,   Joint::Tibia),
  std::tuple(Leg::RightMiddle, Joint::Femur),
  std::tuple(Leg::RightMiddle, Joint::Tibia),
  std::tuple(Leg::RightFront,  Joint::Femur),
  std::tuple(Leg::RightFront,  Joint::Tibia),
};

#endif /* CONST_H_ */
