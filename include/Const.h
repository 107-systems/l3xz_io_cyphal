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

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Joint
{
  Coxa, Femur, Tibia
};

enum class Leg
{
  LeftFront,
  LeftMiddle,
  LeftBack,
  RightFront,
  RightMiddle,
  RightBack
};

typedef struct
{
  float linear_velocity_x;
  float linear_velocity_y;
  float angular_velocity_head_tilt;
  float angular_velocity_head_pan;
  float angular_velocity_z;
} TeleopCommandData;

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

static std::list<Leg> const LEG_LIST =
{
  Leg::LeftFront, Leg::LeftMiddle, Leg::LeftBack, Leg::RightFront, Leg::RightMiddle, Leg::RightBack
};

#endif /* CONST_H_ */
