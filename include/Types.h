/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef TYPES_H_
#define TYPES_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <tuple>

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

typedef std::tuple<Leg, Joint> LegJointPair;

typedef struct
{
  float linear_velocity_x;
  float linear_velocity_y;
  float angular_velocity_head_tilt;
  float angular_velocity_head_pan;
  float angular_velocity_z;
} TeleopCommandData;

#endif /* TYPES_H_ */
